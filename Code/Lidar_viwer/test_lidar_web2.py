"""MS200をRaspberry Piで読み、Windowsのブラウザに表示する1ファイル版。"""

from __future__ import annotations

import threading
import time
import math

import cv2
import numpy as np
import serial
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, StreamingResponse


# ---------- 設定 ----------

SERIAL_PORT = "/dev/ttyAMA0"  # 環境によっては /dev/serial0
BAUD = 230400
WEB_PORT = 8000

WIDTH = 600
HEIGHT = 600
REAL_SIZE_MM = 1000.0
# 角度ごとに最新の1点だけ保持する。点の列を延々と追加しないため遅延しない。
ANGLE_BINS = 360
# 一周の途中で点が消えてちらつかないよう、少しだけ表示を保持する。
POINT_MAX_AGE_SECONDS = 0.30
# LiDAR直近の機体反射を除外する（100 mm以下）。
MIN_DISTANCE_MM = 100
# 機体の柱が45度方向を中心に4本ある想定。
# 43～47度と、それを90度ずつ回転した範囲を除外する。
PILLAR_ANGLE_MIN_DEG = 43.0
PILLAR_ANGLE_MAX_DEG = 47.0
JPEG_QUALITY = 75
WEB_FPS = 15

HEADER = 0x54
FRAME_TYPE = 0x2C
FRAME_SIZE = 47
START_CMD = b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2"


# ---------- ブラウザ表示 ----------

class Viewer:
    def __init__(self) -> None:
        self.app = FastAPI()
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.version = 0
        self.server: uvicorn.Server | None = None
        self.thread: threading.Thread | None = None

        self.app.add_api_route("/", self.index, response_class=HTMLResponse)
        self.app.add_api_route("/image", self.image)
        self.app.add_api_route("/stream", self.stream)

    def start(self) -> None:
        config = uvicorn.Config(
            self.app,
            host="0.0.0.0",
            port=WEB_PORT,
            log_level="warning",
            access_log=False,
        )
        self.server = uvicorn.Server(config)
        self.thread = threading.Thread(target=self.server.run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.server:
            self.server.should_exit = True
        if self.thread:
            self.thread.join(timeout=3)

    def update(self, image: np.ndarray) -> None:
        """OpenCV画像をJPEGに変換して、最新の1枚だけを保持する。"""
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        )
        if not ok:
            raise RuntimeError("JPEG encode failed")
        with self.condition:
            self.jpeg = encoded.tobytes()
            self.version += 1
            self.condition.notify_all()

    def image(self) -> Response:
        with self.condition:
            jpeg = self.jpeg
        if jpeg is None:
            return Response(status_code=404)
        return Response(
            jpeg,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    def stream(self) -> StreamingResponse:
        """最新フレームだけをMJPEGで連続送信する。古い画像は蓄積しない。"""
        def frames():
            last_version = -1
            while True:
                with self.condition:
                    self.condition.wait_for(
                        lambda: self.version != last_version,
                        timeout=5.0,
                    )
                    jpeg = self.jpeg
                    last_version = self.version

                if jpeg is None:
                    continue

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode("ascii") + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "Pragma": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )

    def index(self) -> HTMLResponse:
        return HTMLResponse(
            f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>MS200 LiDAR</title>
  <style>
    body {{ background:#111; color:white; font-family:sans-serif; text-align:center; }}
    img {{ width:min(90vw, 700px); border:1px solid #555; background:black; }}
  </style>
</head>
<body>
  <h1>MS200 LiDAR</h1>
  <img id="lidar" src="/stream" alt="LiDAR image">
</body>
</html>""",
            headers={"Cache-Control": "no-store"},
        )


# ---------- MS200フレーム受信 ----------

def crc8(data: bytes | bytearray) -> int:
    """MS200のCRC-8（元コードのテーブルと同じ計算）。"""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def try_get_frame(ser: serial.Serial, buffer: bytearray) -> bytes | None:
    # 受信済みフレームを先に処理する。毎回readすると、UARTの受信速度が
    # 処理速度を上回った場合に古いフレームがバッファへ溜まり続ける。
    if len(buffer) < FRAME_SIZE:
        buffer.extend(ser.read(512))

    while True:
        index = buffer.find(bytes((HEADER, FRAME_TYPE)))
        if index < 0:
            # 最後の0x54だけは、次回の先頭につながる可能性があるため残す。
            if buffer and buffer[-1] == HEADER:
                buffer[:] = buffer[-1:]
            else:
                buffer.clear()
            return None

        if index:
            del buffer[:index]

        if len(buffer) < FRAME_SIZE:
            return None

        frame = bytes(buffer[:FRAME_SIZE])
        if crc8(frame[:-1]) == frame[-1]:
            del buffer[:FRAME_SIZE]
            return frame

        # CRC不一致なら1バイトずらして、次のヘッダを探す。
        del buffer[0]


def decode_frame(frame: bytes) -> tuple[np.ndarray, np.ndarray]:
    start_angle = int.from_bytes(frame[4:6], "little") / 100.0
    end_angle = int.from_bytes(frame[42:44], "little") / 100.0
    angle_width = (end_angle - start_angle) % 360.0

    raw = np.frombuffer(frame[6:42], dtype=np.uint8).reshape(12, 3)
    distances = raw[:, 0].astype(np.uint16) | (raw[:, 1].astype(np.uint16) << 8)
    # LiDARから10 cm以内の点は機体自身の反射として除外する。
    valid = distances > MIN_DISTANCE_MM
    indexes = np.arange(12)[valid]
    distances = distances[valid]

    angles_deg = (start_angle + angle_width * indexes / 11.0) % 360.0

    # 4方向の柱を除外する。
    # angle % 90 により、43～47、133～137、223～227、313～317度が対象になる。
    angle_in_quadrant = angles_deg % 90.0
    outside_pillars = (
        (angle_in_quadrant < PILLAR_ANGLE_MIN_DEG)
        | (angle_in_quadrant > PILLAR_ANGLE_MAX_DEG)
    )
    angles_deg = angles_deg[outside_pillars]
    distances = distances[outside_pillars]

    angles = np.deg2rad(angles_deg)
    xs = distances * np.sin(angles)
    ys = distances * np.cos(angles)

    half = REAL_SIZE_MM / 2
    visible = (np.abs(xs) <= half) & (np.abs(ys) <= half)
    return xs[visible], ys[visible]


# ---------- 描画 ----------

def create_image(points: list[tuple[float, float]]) -> np.ndarray:
    image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    center_x = WIDTH // 2
    center_y = HEIGHT // 2
    pixels_per_mm = WIDTH / REAL_SIZE_MM

    # 100 mmグリッド
    for mm in np.arange(-REAL_SIZE_MM / 2, REAL_SIZE_MM / 2 + 1, 100):
        px = round(center_x + mm * pixels_per_mm)
        py = round(center_y - mm * pixels_per_mm)
        if 0 <= px < WIDTH:
            cv2.line(image, (px, 0), (px, HEIGHT - 1), (35, 35, 35), 1)
        if 0 <= py < HEIGHT:
            cv2.line(image, (0, py), (WIDTH - 1, py), (35, 35, 35), 1)

    for x, y in points:
        px = round(center_x + x * pixels_per_mm)
        py = round(center_y - y * pixels_per_mm)
        if 0 <= px < WIDTH and 0 <= py < HEIGHT:
            cv2.circle(image, (px, py), 2, (0, 255, 0), -1, cv2.LINE_AA)

    cv2.circle(image, (center_x, center_y), 4, (0, 0, 255), -1)
    cv2.arrowedLine(
        image, (center_x, center_y), (center_x, center_y - 50), (255, 255, 0), 2
    )
    return image


# ---------- メイン処理 ----------

def main() -> None:
    viewer = Viewer()
    viewer.start()
    print(f"Windowsのブラウザで http://<Raspberry PiのIP>:{WEB_PORT}/ を開いてください")

    ser: serial.Serial | None = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.01)
        # 起動前にOS側へ溜まっていた古い受信データを捨てる。
        ser.reset_input_buffer()
        ser.write(START_CMD)
        ser.flush()

        buffer = bytearray()
        # 各角度には最新の測定点を1つだけ保存する。
        # (x, y, 取得時刻) または None
        scan_points: list[tuple[float, float, float] | None] = [None] * ANGLE_BINS
        next_update = 0.0

        while True:
            frame = try_get_frame(ser, buffer)
            if frame is None:
                time.sleep(0.001)
                continue

            xs, ys = decode_frame(frame)
            measured_at = time.monotonic()
            for x, y in zip(xs, ys):
                x_value = float(x)
                y_value = float(y)
                angle = math.degrees(math.atan2(x_value, y_value)) % 360.0
                index = round(angle * ANGLE_BINS / 360.0) % ANGLE_BINS
                scan_points[index] = (x_value, y_value, measured_at)

            # UARTは高頻度なので、JPEG作成はWEB_FPS回/秒に制限する。
            now = time.monotonic()
            if now >= next_update:
                # 更新されなくなった古い点だけを除外する。正常動作中は一周分が
                # 常に残るため、短い点群の塊が回転して見えるちらつきを防げる。
                visible_points = [
                    (point[0], point[1])
                    for point in scan_points
                    if point is not None and now - point[2] <= POINT_MAX_AGE_SECONDS
                ]
                viewer.update(create_image(visible_points))
                next_update = now + 1.0 / WEB_FPS

    except KeyboardInterrupt:
        print("終了します")
    finally:
        if ser and ser.is_open:
            ser.close()
        viewer.stop()


if __name__ == "__main__":
    main()
