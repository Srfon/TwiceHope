#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS200 LiDARだけで前方監視・平行補正・30 cm走行を行う安全テスト。"""

from __future__ import annotations

import argparse
import math
import signal
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import serial
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import HTMLResponse, StreamingResponse

from sts3215 import STS3215


MS200_START_COMMAND = bytes.fromhex("A5 F5 A2 C1 01 81 B3 31 F2")
MS200_FRAME_SIZE = 47
MS200_HEADER = bytes((0x54, 0x2C))
POINTS_PER_FRAME = 12

MOTOR_IDS = (1, 2, 3, 4)
LEFT_IDS = (1, 3)
RIGHT_IDS = (2, 4)
MOTOR_DIRECTION = {1: 1, 2: -1, 3: 1, 4: -1}

WHEEL_DIAMETER_MM = 80.0
ENCODER_COUNTS = 4096
ROBOT_RADIUS_MM = 40.0

NAV_MIN_MM = 100.0       # 描画・Houghから消す近距離点
NAV_MAX_MM = 2500.0
TARGET_CLEARANCE_MM = 100.0      # 実機調整で良好だった停止目標
STOP_COMMAND_CLEARANCE_MM = 80.0 # 実測3 cmの惰性・通信遅延を先読み
SLOWDOWN_CLEARANCE_MM = 200.0
RUN_DISTANCE_MM = 300.0
FRONT_HALF_WIDTH_MM = 90.0
LIDAR_STALE_SECONDS = 0.5
POINT_LIFETIME_SECONDS = 0.35
RENDER_INTERVAL_SECONDS = 1 / 15
SEGMENT_PAUSE_SECONDS = 0.5
PROGRAM_VERSION = "LiDAR continuous v5"

BASE_POWER = 2000
NEAR_WALL_POWER = 750
MIN_POWER = 700
MAX_POWER = 1200
KP_STEERING = 10.0
MAX_STEERING = 220

IMAGE_SIZE = 600
MM_PER_PIXEL = 5.0
ROBOT_PIXEL = (IMAGE_SIZE // 2, int(IMAGE_SIZE * 0.78))


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x4D) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def signed_encoder_delta(current: int, previous: int) -> int:
    """0..4095のエンコーダー値をまたいだ差分を-2048..2047へ直す。"""
    return ((current - previous + 2048) % 4096) - 2048


@dataclass(frozen=True)
class Scan:
    points: np.ndarray
    timestamp: float


class MS200:
    def __init__(self, port: str, baudrate: int, angle_offset_deg: float):
        self.port = port
        self.baudrate = baudrate
        self.angle_offset_deg = angle_offset_deg
        self._serial: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._bins: list[Optional[tuple[float, float, float, float]]] = [None] * 360
        self._last_frame_time = 0.0
        self.error: Optional[BaseException] = None

    def start(self) -> None:
        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.05)
        self._serial.reset_input_buffer()
        self._serial.write(MS200_START_COMMAND)
        self._serial.flush()
        self._thread = threading.Thread(target=self._run, name="ms200", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def _read_frame(self) -> Optional[bytes]:
        assert self._serial is not None
        while not self._stop.is_set():
            first = self._serial.read(1)
            if first != MS200_HEADER[:1]:
                continue
            second = self._serial.read(1)
            if second != MS200_HEADER[1:]:
                continue
            rest = self._serial.read(MS200_FRAME_SIZE - 2)
            if len(rest) != MS200_FRAME_SIZE - 2:
                continue
            frame = MS200_HEADER + rest
            if crc8(frame[:-1]) == frame[-1]:
                return frame
        return None

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                frame = self._read_frame()
                if frame is None:
                    continue
                start_deg = int.from_bytes(frame[4:6], "little") * 0.01
                end_deg = int.from_bytes(frame[42:44], "little") * 0.01
                span = (end_deg - start_deg) % 360.0
                now = time.monotonic()
                updates: list[tuple[int, float, float, float]] = []
                for index in range(POINTS_PER_FRAME):
                    pos = 6 + index * 3
                    distance = float(int.from_bytes(frame[pos:pos + 2], "little"))
                    intensity = float(frame[pos + 2])
                    angle = (start_deg + span * index / (POINTS_PER_FRAME - 1) + self.angle_offset_deg) % 360.0
                    if distance > 0:
                        updates.append((int(round(angle)) % 360, angle, distance, intensity))
                with self._lock:
                    for bin_index, angle, distance, intensity in updates:
                        self._bins[bin_index] = (angle, distance, intensity, now)
                    self._last_frame_time = now
        except BaseException as exc:
            self.error = exc
            self._stop.set()

    def snapshot(self) -> Scan:
        with self._lock:
            now = time.monotonic()
            rows = [item[:3] for item in self._bins if item is not None and now - item[3] <= POINT_LIFETIME_SECONDS]
            timestamp = self._last_frame_time
        if not rows:
            return Scan(np.empty((0, 3), dtype=np.float32), timestamp)
        return Scan(np.asarray(rows, dtype=np.float32), timestamp)


class Motors:
    def __init__(self, port: str):
        self.servo = STS3215(port=port, baudrate=1_000_000, timeout=0.05)
        self._last_command: Optional[tuple[int, int]] = None

    def start(self) -> None:
        self.servo.open()
        for motor_id in MOTOR_IDS:
            if not self.servo.ping(motor_id):
                raise RuntimeError(f"モーターID={motor_id}が応答しません")
        for motor_id in MOTOR_IDS:
            self.servo.set_mode(motor_id, self.servo.MODE_VELOCITY)
            self.servo.torque(motor_id, True)
        self.stop(force=True)

    @staticmethod
    def _raw_speed(motor_id: int, speed: int) -> int:
        signed = int(speed) * MOTOR_DIRECTION[motor_id]
        return abs(signed) if signed >= 0 else abs(signed) | 0x8000

    def drive(self, left: int, right: int, force: bool = False) -> None:
        command = (int(left), int(right))
        if not force and command == self._last_command:
            return
        for motor_id in LEFT_IDS:
            self.servo.write_u16(motor_id, self.servo.ADDR_GOAL_SPEED, self._raw_speed(motor_id, left))
        for motor_id in RIGHT_IDS:
            self.servo.write_u16(motor_id, self.servo.ADDR_GOAL_SPEED, self._raw_speed(motor_id, right))
        self._last_command = command

    def stop(self, force: bool = False) -> None:
        self.drive(0, 0, force=force)

    def encoder_pair(self) -> tuple[int, int]:
        left = self.servo.read_u16(1, self.servo.ADDR_PRESENT_POSITION)
        right = self.servo.read_u16(2, self.servo.ADDR_PRESENT_POSITION)
        return left, right

    def close(self) -> None:
        try:
            self.stop(force=True)
            for motor_id in MOTOR_IDS:
                self.servo.torque(motor_id, False)
        finally:
            self.servo.close()


def to_xy(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float32)
    radians = np.deg2rad(points[:, 0])
    x = points[:, 1] * np.sin(radians)
    y = points[:, 1] * np.cos(radians)
    return np.column_stack((x, y)).astype(np.float32)


def pillar_angle(angle: float) -> bool:
    """45, 135, 225, 315度を中心に、左右1度ずつ（合計2度）除く。"""
    return any(abs(((angle - center + 180.0) % 360.0) - 180.0) <= 1.0 for center in (45, 135, 225, 315))


def navigation_points(scan: Scan) -> np.ndarray:
    if len(scan.points) == 0:
        return scan.points
    keep = (scan.points[:, 1] >= NAV_MIN_MM) & (scan.points[:, 1] <= NAV_MAX_MM)
    keep &= np.asarray([not pillar_angle(float(a)) for a in scan.points[:, 0]])
    return scan.points[keep]


def make_binary(points: np.ndarray) -> np.ndarray:
    image = np.zeros((IMAGE_SIZE, IMAGE_SIZE), dtype=np.uint8)
    for x_mm, y_mm in to_xy(points):
        px = int(round(ROBOT_PIXEL[0] + x_mm / MM_PER_PIXEL))
        py = int(round(ROBOT_PIXEL[1] - y_mm / MM_PER_PIXEL))
        if 2 <= px < IMAGE_SIZE - 2 and 2 <= py < IMAGE_SIZE - 2:
            cv2.circle(image, (px, py), 2, 255, -1)
    # 点をつないだ後に収縮し、太すぎる輪郭と孤立ノイズを抑える。
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, close_kernel, iterations=1)
    return cv2.erode(image, erode_kernel, iterations=1)


def hough_lines(binary: np.ndarray) -> list[tuple[int, int, int, int]]:
    edges = cv2.Canny(binary, 40, 120)
    found = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=22, minLineLength=35, maxLineGap=18)
    if found is None:
        return []
    return [tuple(int(v) for v in line[0]) for line in found]


def parallel_error_deg(lines: list[tuple[int, int, int, int]]) -> Optional[float]:
    errors: list[float] = []
    for x1, y1, x2, y2 in lines:
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 40:
            continue
        error = math.degrees(math.atan2(dx, -dy))
        while error > 90:
            error -= 180
        while error < -90:
            error += 180
        if abs(error) <= 35:
            errors.append(error)
    return float(np.median(errors)) if errors else None


def front_wall_clearance_mm(lines: list[tuple[int, int, int, int]]) -> Optional[float]:
    """ロボット正面を横切る、進行方向にほぼ直角な壁までの隙間を返す。"""
    candidates: list[float] = []
    corridor_px = FRONT_HALF_WIDTH_MM / MM_PER_PIXEL
    robot_x, robot_y = ROBOT_PIXEL
    for x1, y1, x2, y2 in lines:
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 35 or abs(dx) < 1:
            continue
        # 正面を向いた壁は画像上でほぼ水平になる。
        wall_angle = math.degrees(math.atan2(dy, dx))
        while wall_angle > 90:
            wall_angle -= 180
        while wall_angle < -90:
            wall_angle += 180
        if abs(wall_angle) > 18:
            continue
        left, right = sorted((x1, x2))
        if right < robot_x - corridor_px or left > robot_x + corridor_px:
            continue
        # 直線をロボット中心Xまで延長し、前方距離を求める。
        y_at_robot = y1 + (robot_x - x1) * dy / dx
        center_distance = (robot_y - y_at_robot) * MM_PER_PIXEL
        clearance = center_distance - ROBOT_RADIUS_MM
        if clearance > 0:
            candidates.append(clearance)
    return min(candidates) if candidates else None


def process_and_render(scan: Scan, travelled_mm: float, command: str) -> tuple[bytes, Optional[float], Optional[float]]:
    """除去、モルフォロジー、Hough、距離判断、描画を一括して行う。"""
    nav = navigation_points(scan)
    binary = make_binary(nav)
    lines = hough_lines(binary)
    error = parallel_error_deg(lines)
    wall_clearance = front_wall_clearance_mm(lines)
    canvas = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    for x1, y1, x2, y2 in lines:
        cv2.line(canvas, (x1, y1), (x2, y2), (0, 200, 255), 2)
    cv2.circle(canvas, ROBOT_PIXEL, max(3, int(ROBOT_RADIUS_MM / MM_PER_PIXEL)), (0, 0, 255), 2)
    cv2.line(canvas, ROBOT_PIXEL, (ROBOT_PIXEL[0], ROBOT_PIXEL[1] - 30), (0, 0, 255), 2)
    wall_text = "---" if wall_clearance is None else f"{wall_clearance / 10:.1f} cm"
    error_text = "---" if error is None else f"{error:+.1f} deg"
    cv2.putText(canvas, f"processed wall: {wall_text}", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    cv2.putText(canvas, f"line: {error_text}", (12, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    cv2.putText(canvas, f"travel: {travelled_mm / 10:.1f} cm", (12, 79), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    cv2.putText(canvas, command, (12, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
    cv2.putText(canvas, PROGRAM_VERSION, (12, IMAGE_SIZE - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
    ok, encoded = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if not ok:
        raise RuntimeError("JPEGエンコードに失敗しました")
    return encoded.tobytes(), error, wall_clearance


class SharedView:
    def __init__(self):
        self.lock = threading.Lock()
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Waiting for MS200 LiDAR...", (55, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        ok, encoded = cv2.imencode(".jpg", placeholder, [cv2.IMWRITE_JPEG_QUALITY, 75])
        self.frame: Optional[bytes] = encoded.tobytes() if ok else None

    def update(self, frame: bytes) -> None:
        with self.lock:
            self.frame = frame

    def get(self) -> Optional[bytes]:
        with self.lock:
            return self.frame


def start_web(view: SharedView, port: int) -> uvicorn.Server:
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return """<!doctype html><meta name=viewport content='width=device-width'>
<style>
html,body{margin:0;width:100%;height:100%;overflow:hidden;background:#111;color:#eee;text-align:center;font-family:sans-serif}
h3{height:5vh;line-height:5vh;margin:0;font-size:clamp(18px,2.2vw,30px)}
img{display:block;width:min(95vw,900px);height:94vh;object-fit:contain;margin:0 auto}
</style>
<h3>MS200 LiDAR</h3><img src='/stream' onerror="this.src='/stream?t='+Date.now()">"""

    @app.get("/snapshot.jpg")
    async def snapshot():
        frame = view.get()
        return Response(content=frame or b"", media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    def frames():
        previous = None
        while True:
            frame = view.get()
            if frame is not None and frame != previous:
                previous = frame
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(1 / 15)

    @app.get("/stream")
    async def stream():
        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    server = uvicorn.Server(uvicorn.Config(app, host="0.0.0.0", port=port, log_level="warning"))
    thread = threading.Thread(target=server.run, name="web", daemon=True)
    thread.start()
    deadline = time.monotonic() + 5.0
    while not server.started:
        if not thread.is_alive():
            raise RuntimeError(f"Webサーバーを起動できません。ポート{port}が使用中でないか確認してください")
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Webサーバーの起動がタイムアウトしました: port={port}")
        time.sleep(0.05)
    return server


def wait_for_lidar(lidar: MS200) -> None:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if lidar.error:
            raise RuntimeError(f"LiDAR受信エラー: {lidar.error}")
        scan = lidar.snapshot()
        if len(scan.points) >= 120 and time.monotonic() - scan.timestamp < 0.3:
            print(f"LiDAR ready: {len(scan.points)} points")
            return
        time.sleep(0.05)
    raise RuntimeError("LiDARデータを5秒以内に取得できません")


def run(args: argparse.Namespace) -> None:
    lidar = MS200(args.lidar_port, args.lidar_baud, args.angle_offset)
    motors: Optional[Motors] = None
    view = SharedView()
    server = start_web(view, args.web_port)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    print(f"{PROGRAM_VERSION} started")
    print(f"Windowsで http://<Raspberry PiのIP>:{args.web_port}/ を開いてください")

    travelled = 0.0
    total_travelled = 0.0
    segment_number = 1
    command = "VIEW ONLY" if args.view_only else "WAIT"
    try:
        lidar.start()
        wait_for_lidar(lidar)
        if not args.view_only:
            motors = Motors(args.motor_port)
            motors.start()
            print("車輪を浮かせるか、前方を40 cm以上空けてください。")
            input("Enterを押すとLiDAR走行を開始します: ")
            previous_left, previous_right = motors.encoder_pair()
            motors.drive(BASE_POWER, BASE_POWER)
            command = "FORWARD"

        last_control = 0.0
        last_render = 0.0
        line_error: Optional[float] = None
        wall_clearance: Optional[float] = None
        pause_until: Optional[float] = None
        while not stopping.is_set():
            now = time.monotonic()
            if lidar.error:
                raise RuntimeError(f"LiDAR受信エラー: {lidar.error}")
            scan = lidar.snapshot()
            if now - scan.timestamp > LIDAR_STALE_SECONDS:
                raise RuntimeError("LiDARデータが停止しました")

            if now - last_render >= RENDER_INTERVAL_SECONDS:
                frame, line_error, wall_clearance = process_and_render(scan, travelled, command)
                view.update(frame)
                last_render = now

            if args.view_only:
                time.sleep(0.01)
                continue
            assert motors is not None
            stop_command_mm = args.stop_command_cm * 10.0
            if wall_clearance is not None and wall_clearance <= stop_command_mm:
                command = "STOP: WALL TARGET 10 cm"
                motors.stop()
                print(f"正面壁を先読み停止: command_at={wall_clearance / 10:.1f} cm, target={TARGET_CLEARANCE_MM / 10:.1f} cm")
                break

            # 30 cmごとの停止中も、Web表示と処理後Hough壁判定を継続する。
            if pause_until is not None:
                if now < pause_until:
                    time.sleep(0.01)
                    continue
                previous_left, previous_right = motors.encoder_pair()
                travelled = 0.0
                segment_number += 1
                pause_until = None
                motors.drive(BASE_POWER, BASE_POWER)
                command = f"FORWARD SEGMENT {segment_number}"
                print(f"segment {segment_number} started, total={total_travelled / 10:.1f} cm")
                last_control = now
                continue

            if now - last_control >= 0.10:
                current_left, current_right = motors.encoder_pair()
                dl = abs(signed_encoder_delta(current_left, previous_left))
                dr = abs(signed_encoder_delta(current_right, previous_right))
                previous_left, previous_right = current_left, current_right
                step_mm = ((dl + dr) / 2.0) * (math.pi * WHEEL_DIAMETER_MM / ENCODER_COUNTS)
                travelled += step_mm
                total_travelled += step_mm
                if travelled >= RUN_DISTANCE_MM:
                    motors.stop()
                    command = f"PAUSE: SEGMENT {segment_number}"
                    pause_until = now + SEGMENT_PAUSE_SECONDS
                    print(f"segment {segment_number} completed: {travelled / 10:.1f} cm, total={total_travelled / 10:.1f} cm")
                    last_control = now
                    continue
                current_base = NEAR_WALL_POWER if wall_clearance is not None and wall_clearance <= SLOWDOWN_CLEARANCE_MM else BASE_POWER
                correction = 0 if line_error is None else int(np.clip(args.steering_sign * KP_STEERING * line_error, -MAX_STEERING, MAX_STEERING))
                left = int(np.clip(current_base - correction, MIN_POWER, MAX_POWER))
                right = int(np.clip(current_base + correction, MIN_POWER, MAX_POWER))
                motors.drive(left, right)
                command = f"FORWARD S{segment_number} L={left} R={right}"
                wall_text = "---" if wall_clearance is None else f"{wall_clearance / 10:.1f}"
                print(f"processed_wall={wall_text} cm travel={travelled / 10:5.1f} cm line={line_error} L={left} R={right}")
                last_control = now
            time.sleep(0.015)

        if not args.view_only:
            print("停止しました。Ctrl+Cで終了します。ブラウザ表示はそのまま確認できます。")
            while not stopping.is_set():
                scan = lidar.snapshot()
                frame, _, _ = process_and_render(scan, travelled, command)
                view.update(frame)
                time.sleep(1 / 15)
    finally:
        if motors is not None:
            motors.close()
        lidar.stop()
        server.should_exit = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--view-only", action="store_true", help="モーターを初期化せずLiDAR表示だけ行う")
    parser.add_argument("--lidar-port", default="/dev/ttyAMA0")
    parser.add_argument("--lidar-baud", type=int, default=230400)
    parser.add_argument("--motor-port", default="/dev/ttyUSB0")
    parser.add_argument("--web-port", type=int, default=8001)
    parser.add_argument("--angle-offset", type=float, default=0.0)
    parser.add_argument("--steering-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        "--stop-command-cm",
        type=float,
        default=STOP_COMMAND_CLEARANCE_MM / 10.0,
        help="停止命令を出す処理後の正面壁距離（実機調整値は既定8 cm）",
    )
    return parser.parse_args()


if __name__ == "__main__":
    try:
        run(parse_args())
    except KeyboardInterrupt:
        pass
    except BaseException as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}")
        raise
