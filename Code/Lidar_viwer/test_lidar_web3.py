#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS200の5段階画像処理をWindowsブラウザへ表示する1ファイル版。"""

from __future__ import annotations

import argparse
import math
import threading
import time

import cv2
import numpy as np
import serial
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, StreamingResponse


# ---------- 設定 ----------

SERIAL_PORT = "/dev/ttyAMA0"
BAUD = 230400
DEFAULT_WEB_PORT = 8000

SOURCE_SIZE = 600
REAL_SIZE_MM = 1000.0
ANGLE_BINS = 360
POINT_MAX_AGE_SECONDS = 0.30
MAX_DISTANCE_MM = 2500.0
CENTER_EXCLUSION_HALF_SIZE_MM = 100.0

CLOSE_KERNEL_SIZE = 7
ISOLATED_OPEN_LENGTH = 11
ISOLATED_OPEN_THICKNESS = 3
ERODE_KERNEL_SIZE = 3
# 同一直線上にある壁点をつなぐ最大隙間。1000 mmを600 pxで描くため約72 px。
WALL_CONNECT_GAP_MM = 120.0
HOUGH_THRESHOLD = 30
HOUGH_MIN_LINE_LENGTH = 45
HOUGH_MAX_LINE_GAP = 12

# 4隅に散らばる短い塊を除去し、長い壁成分を残す。
NOISE_MIN_AREA = 20
NOISE_MIN_LONG_SIDE = 28
NOISE_MIN_ASPECT_RATIO = 2.8
NOISE_KEEP_LARGE_AREA = 150

PANEL_SIZE = 360
PANEL_HEADER = 36
JPEG_QUALITY = 75
WEB_FPS = 15
PROGRAM_VERSION = "MS200 pipeline viewer v16 morphology opening"

HEADER = 0x54
FRAME_TYPE = 0x2C
FRAME_SIZE = 47
START_CMD = b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2"


# ---------- ブラウザ表示 ----------

class Viewer:
    def __init__(self, web_port: int) -> None:
        self.web_port = web_port
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
            port=self.web_port,
            log_level="warning",
            access_log=False,
        )
        self.server = uvicorn.Server(config)
        self.thread = threading.Thread(target=self.server.run, name="web", daemon=True)
        self.thread.start()

        deadline = time.monotonic() + 5.0
        while not self.server.started:
            if not self.thread.is_alive():
                raise RuntimeError(
                    f"Webサーバーを起動できません。ポート{self.web_port}が使用中でないか確認してください"
                )
            if time.monotonic() >= deadline:
                raise RuntimeError("Webサーバーの起動がタイムアウトしました")
            time.sleep(0.05)

    def stop(self) -> None:
        if self.server:
            self.server.should_exit = True
        if self.thread:
            self.thread.join(timeout=3)

    def update(self, image: np.ndarray) -> None:
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
            """<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>MS200 Processing Pipeline</title>
  <style>
    html,body { margin:0; width:100%; min-height:100%; background:#111; color:#eee;
      font-family:sans-serif; text-align:center; }
    h1 { height:5vh; line-height:5vh; margin:0; font-size:clamp(18px,2.2vw,30px); }
    img { display:block; width:min(98vw,1400px); height:94vh; object-fit:contain;
      margin:0 auto; background:#080808; }
  </style>
</head>
<body>
  <h1>MS200 LiDAR Processing Pipeline</h1>
  <img id="lidar" src="/stream" alt="LiDAR processing dashboard">
</body>
</html>""",
            headers={"Cache-Control": "no-store"},
        )


# ---------- MS200フレーム受信 ----------

def crc8(data: bytes | bytearray) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def try_get_frame(ser: serial.Serial, buffer: bytearray) -> bytes | None:
    if len(buffer) < FRAME_SIZE:
        buffer.extend(ser.read(512))

    while True:
        index = buffer.find(bytes((HEADER, FRAME_TYPE)))
        if index < 0:
            buffer[:] = buffer[-1:] if buffer and buffer[-1] == HEADER else b""
            return None
        if index:
            del buffer[:index]
        if len(buffer) < FRAME_SIZE:
            return None

        frame = bytes(buffer[:FRAME_SIZE])
        if crc8(frame[:-1]) == frame[-1]:
            del buffer[:FRAME_SIZE]
            return frame
        del buffer[0]


def decode_frame(frame: bytes) -> tuple[np.ndarray, np.ndarray]:
    """距離・柱をまだ除去せず、角度と距離を返す。"""
    start_angle = int.from_bytes(frame[4:6], "little") / 100.0
    end_angle = int.from_bytes(frame[42:44], "little") / 100.0
    angle_width = (end_angle - start_angle) % 360.0
    raw = np.frombuffer(frame[6:42], dtype=np.uint8).reshape(12, 3)
    distances = raw[:, 0].astype(np.uint16) | (raw[:, 1].astype(np.uint16) << 8)
    indexes = np.arange(12, dtype=np.float32)
    angles_deg = (start_angle + angle_width * indexes / 11.0) % 360.0
    valid = distances > 0
    return angles_deg[valid].astype(np.float32), distances[valid].astype(np.float32)


# ---------- 画像処理 ----------

def polar_to_xy(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float32)
    angles = np.deg2rad(points[:, 0])
    return np.column_stack(
        (points[:, 1] * np.sin(angles), points[:, 1] * np.cos(angles))
    ).astype(np.float32)


def filter_navigation_points(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return points
    xy = polar_to_xy(points)
    inside_center_square = (
        (np.abs(xy[:, 0]) <= CENTER_EXCLUSION_HALF_SIZE_MM)
        & (np.abs(xy[:, 1]) <= CENTER_EXCLUSION_HALF_SIZE_MM)
    )
    keep = (~inside_center_square) & (points[:, 1] <= MAX_DISTANCE_MM)
    return points[keep]


def points_to_binary(points: np.ndarray) -> np.ndarray:
    image = np.zeros((SOURCE_SIZE, SOURCE_SIZE), dtype=np.uint8)
    center = SOURCE_SIZE // 2
    pixels_per_mm = SOURCE_SIZE / REAL_SIZE_MM
    half = REAL_SIZE_MM / 2.0
    for x, y in polar_to_xy(points):
        if abs(x) > half or abs(y) > half:
            continue
        px = int(round(center + x * pixels_per_mm))
        py = int(round(center - y * pixels_per_mm))
        if 2 <= px < SOURCE_SIZE - 2 and 2 <= py < SOURCE_SIZE - 2:
            cv2.circle(image, (px, py), 2, 255, -1)
    return image


def add_grid(image: np.ndarray) -> np.ndarray:
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    center = SOURCE_SIZE // 2
    pixels_per_mm = SOURCE_SIZE / REAL_SIZE_MM
    for mm in np.arange(-REAL_SIZE_MM / 2, REAL_SIZE_MM / 2 + 1, 100):
        px = round(center + mm * pixels_per_mm)
        py = round(center - mm * pixels_per_mm)
        if 0 <= px < SOURCE_SIZE:
            cv2.line(color, (px, 0), (px, SOURCE_SIZE - 1), (35, 35, 35), 1)
        if 0 <= py < SOURCE_SIZE:
            cv2.line(color, (0, py), (SOURCE_SIZE - 1, py), (35, 35, 35), 1)
    color[image > 0] = (0, 255, 0)
    cv2.circle(color, (center, center), 4, (0, 0, 255), -1)
    cv2.arrowedLine(color, (center, center), (center, center - 50), (255, 255, 0), 2)
    return color


def hough_lines(binary: np.ndarray) -> list[tuple[int, int, int, int]]:
    """収縮後の二値画像から直接、確率的Hough変換で線分を抽出する。"""
    found = cv2.HoughLinesP(
        binary,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=HOUGH_MIN_LINE_LENGTH,
        maxLineGap=HOUGH_MAX_LINE_GAP,
    )
    if found is None:
        return []
    normalized = np.asarray(found).reshape(-1, 4)
    return [tuple(int(value) for value in line) for line in normalized]


def connect_wall_points(binary: np.ndarray) -> np.ndarray:
    """縦・横方向で同一直線上にある壁点の隙間をクロージングで補間する。"""
    pixels_per_mm = SOURCE_SIZE / REAL_SIZE_MM
    gap_pixels = max(3, int(round(WALL_CONNECT_GAP_MM * pixels_per_mm)))
    if gap_pixels % 2 == 0:
        gap_pixels += 1
    vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, gap_pixels))
    horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (gap_pixels, 3))
    vertical = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, vertical_kernel)
    horizontal = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, horizontal_kernel)
    return cv2.bitwise_or(binary, cv2.bitwise_or(vertical, horizontal))


def remove_scattered_components(binary: np.ndarray) -> np.ndarray:
    """短く丸い散乱点を除去し、細長い壁または十分大きい成分を残す。"""
    cleaned = np.zeros_like(binary)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    for label in range(1, count):
        x, y, width, height, area = stats[label]
        long_side = max(int(width), int(height))
        short_side = max(1, min(int(width), int(height)))
        aspect_ratio = long_side / short_side
        wall_shaped = (
            area >= NOISE_MIN_AREA
            and long_side >= NOISE_MIN_LONG_SIDE
            and aspect_ratio >= NOISE_MIN_ASPECT_RATIO
        )
        # 面積だけが大きい丸い塊は残さず、長さもある成分だけ救済する。
        large_component = (
            area >= NOISE_KEEP_LARGE_AREA
            and long_side >= NOISE_MIN_LONG_SIDE
        )
        if wall_shaped or large_component:
            cleaned[labels == label] = 255
    return cleaned


def make_panel(image: np.ndarray, title: str) -> np.ndarray:
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image
    resized = cv2.resize(color, (PANEL_SIZE, PANEL_SIZE), interpolation=cv2.INTER_AREA)
    panel = np.full((PANEL_SIZE + PANEL_HEADER, PANEL_SIZE, 3), 18, dtype=np.uint8)
    panel[PANEL_HEADER:] = resized
    cv2.putText(panel, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2)
    cv2.rectangle(panel, (0, 0), (PANEL_SIZE - 1, PANEL_SIZE + PANEL_HEADER - 1), (110, 110, 110), 2)
    return panel


def create_dashboard(raw_points: np.ndarray) -> np.ndarray:
    raw_binary = points_to_binary(raw_points)
    filtered = filter_navigation_points(raw_points)
    filtered_binary = points_to_binary(filtered)

    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE)
    )
    vertical_open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (ISOLATED_OPEN_THICKNESS, ISOLATED_OPEN_LENGTH),
    )
    horizontal_open_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (ISOLATED_OPEN_LENGTH, ISOLATED_OPEN_THICKNESS),
    )
    erode_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (ERODE_KERNEL_SIZE, ERODE_KERNEL_SIZE)
    )
    locally_closed = cv2.morphologyEx(filtered_binary, cv2.MORPH_CLOSE, close_kernel)
    # 壁接続より先に、1～2点程度の孤立した小さな塊をオープニングで消す。
    vertical_opened = cv2.morphologyEx(
        locally_closed,
        cv2.MORPH_OPEN,
        vertical_open_kernel,
        iterations=1,
    )
    horizontal_opened = cv2.morphologyEx(
        locally_closed,
        cv2.MORPH_OPEN,
        horizontal_open_kernel,
        iterations=1,
    )
    without_isolated_points = cv2.bitwise_or(vertical_opened, horizontal_opened)
    closed = connect_wall_points(without_isolated_points)
    eroded = cv2.erode(closed, erode_kernel, iterations=1)
    cleaned = remove_scattered_components(eroded)
    lines = hough_lines(cleaned)

    hough_image = cv2.cvtColor(cleaned, cv2.COLOR_GRAY2BGR)
    for x1, y1, x2, y2 in lines:
        cv2.line(hough_image, (x1, y1), (x2, y2), (0, 200, 255), 2)
    cv2.putText(
        hough_image,
        f"lines: {len(lines)}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        hough_image,
        PROGRAM_VERSION,
        (12, SOURCE_SIZE - 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (180, 180, 180),
        1,
    )

    stages = [
        ("1. Raw (Unfiltered)", add_grid(raw_binary)),
        ("2. After 20x20 cm Filter", add_grid(filtered_binary)),
        ("3. Opening + Closing", closed),
        ("4. After Erosion", eroded),
        ("5. Noise Filter", cleaned),
        ("6. Hough Lines", hough_image),
    ]
    cell_height = PANEL_SIZE + PANEL_HEADER
    dashboard = np.full((cell_height * 2, PANEL_SIZE * 3, 3), 8, dtype=np.uint8)
    for index, (title, image) in enumerate(stages):
        row, column = divmod(index, 3)
        y = row * cell_height
        x = column * PANEL_SIZE
        dashboard[y:y + cell_height, x:x + PANEL_SIZE] = make_panel(image, title)
    return dashboard


# ---------- メイン処理 ----------

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", default=SERIAL_PORT)
    parser.add_argument("--web-port", type=int, default=DEFAULT_WEB_PORT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    viewer = Viewer(args.web_port)
    viewer.start()
    print(f"{PROGRAM_VERSION} started")
    print(f"Windowsのブラウザで http://<Raspberry PiのIP>:{args.web_port}/ を開いてください")

    ser: serial.Serial | None = None
    try:
        ser = serial.Serial(args.serial_port, BAUD, timeout=0.01)
        ser.reset_input_buffer()
        ser.write(START_CMD)
        ser.flush()

        buffer = bytearray()
        # (角度deg, 距離mm, 取得時刻)を角度ごとに1点だけ保持する。
        scan_points: list[tuple[float, float, float] | None] = [None] * ANGLE_BINS
        next_update = 0.0

        while True:
            frame = try_get_frame(ser, buffer)
            if frame is None:
                time.sleep(0.001)
                continue

            angles, distances = decode_frame(frame)
            measured_at = time.monotonic()
            for angle, distance in zip(angles, distances):
                index = round(float(angle) * ANGLE_BINS / 360.0) % ANGLE_BINS
                scan_points[index] = (float(angle), float(distance), measured_at)

            now = time.monotonic()
            if now >= next_update:
                visible = [
                    (point[0], point[1])
                    for point in scan_points
                    if point is not None and now - point[2] <= POINT_MAX_AGE_SECONDS
                ]
                raw_points = (
                    np.asarray(visible, dtype=np.float32)
                    if visible
                    else np.empty((0, 2), dtype=np.float32)
                )
                viewer.update(create_dashboard(raw_points))
                next_update = now + 1.0 / WEB_FPS

    except KeyboardInterrupt:
        print("終了します")
    finally:
        if ser and ser.is_open:
            ser.close()
        viewer.stop()


if __name__ == "__main__":
    main()
