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

# LiDAR画像処理（ms200_pipeline_viewer.py v20を内蔵）
SOURCE_SIZE = 600
REAL_SIZE_MM = 1000.0
MAX_DISTANCE_MM = 2500.0
CENTER_EXCLUSION_HALF_SIZE_MM = 100.0
PROTECT_LINE_THRESHOLD = 12
PROTECT_LINE_MIN_LENGTH = 24
PROTECT_LINE_MAX_GAP = 10
PROTECT_LINE_THICKNESS = 7
CLOSE_KERNEL_SIZE = 5
ERODE_KERNEL_SIZE = 3
HOUGH_THRESHOLD = 22
HOUGH_MIN_LINE_LENGTH = 35
HOUGH_MAX_LINE_GAP = 18
NOISE_MIN_AREA = 10
NOISE_MIN_PRINCIPAL_SPAN = 15.0
NOISE_MIN_LINEARITY = 1.8
NOISE_KEEP_LARGE_AREA = 100
PANEL_SIZE = 360
PANEL_HEADER = 36
JPEG_QUALITY = 70
WEB_FPS = 20

TARGET_CLEARANCE_MM = 125.0      # 実機調整で良好だった停止目標
STOP_COMMAND_CLEARANCE_MM = 80.0 # 実測3 cmの惰性・通信遅延を先読み
SLOWDOWN_CLEARANCE_MM = 200.0
RUN_DISTANCE_MM = 300.0
FRONT_HALF_WIDTH_MM = 90.0
SIDE_TARGET_CLEARANCE_MM = 100.0
SIDE_MAX_DETECTION_CLEARANCE_MM = 250.0
SIDE_WALL_LOOKAHEAD_MM = 250.0
SIDE_GUIDANCE_LOOKAHEAD_MM = 150.0
SIDE_MAX_PARALLEL_ANGLE_DEG = 45.0
SIDE_EMERGENCY_CLEARANCE_MM = 20.0
LIDAR_STALE_SECONDS = 0.5
POINT_LIFETIME_SECONDS = 0.35
RENDER_INTERVAL_SECONDS = 1 / WEB_FPS
SEGMENT_PAUSE_SECONDS = 0.5
PROGRAM_VERSION = "LiDAR wall line trace v26"

BASE_POWER = 2000
NEAR_WALL_POWER = 750
MIN_POWER = 700
MAX_POWER = 1200
KP_WALL_DISTANCE = 4.0
KP_WALL_ANGLE = 12.0
MAX_STEERING = 220

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


def mix_motor_power(base_power: int, correction: int) -> tuple[int, int]:
    """左右差を消さずに、両輪出力を許容範囲へ平行移動する。"""
    left = float(base_power - correction)
    right = float(base_power + correction)

    # 個別に上限へ丸めると両輪が同じ値になって補正が消えるため、
    # 左右差を保ったまま2輪をまとめて範囲内へ移動する。
    if max(left, right) > MAX_POWER:
        overflow = max(left, right) - MAX_POWER
        left -= overflow
        right -= overflow
    if min(left, right) < MIN_POWER:
        shortage = MIN_POWER - min(left, right)
        left += shortage
        right += shortage

    return (
        int(np.clip(round(left), MIN_POWER, MAX_POWER)),
        int(np.clip(round(right), MIN_POWER, MAX_POWER)),
    )


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


@dataclass(frozen=True)
class PipelineResult:
    """走行判断とWeb表示で共有する各画像処理工程。"""

    raw_binary: np.ndarray
    filtered_binary: np.ndarray
    closed: np.ndarray
    eroded: np.ndarray
    cleaned: np.ndarray
    lines: list[tuple[int, int, int, int]]


def polar_to_xy(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 2), dtype=np.float32)
    angles = np.deg2rad(points[:, 0])
    return np.column_stack(
        (points[:, 1] * np.sin(angles), points[:, 1] * np.cos(angles))
    ).astype(np.float32)


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


def protected_center_line_mask(raw_binary: np.ndarray) -> np.ndarray:
    """中心20×20 cm内で直線を形成する点を保護する。"""
    center = SOURCE_SIZE // 2
    half_pixels = int(round(CENTER_EXCLUSION_HALF_SIZE_MM * SOURCE_SIZE / REAL_SIZE_MM))
    center_roi = np.zeros_like(raw_binary)
    cv2.rectangle(
        center_roi,
        (center - half_pixels, center - half_pixels),
        (center + half_pixels, center + half_pixels),
        255,
        -1,
    )
    center_points = cv2.bitwise_and(raw_binary, center_roi)
    small_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    center_points = cv2.morphologyEx(center_points, cv2.MORPH_CLOSE, small_close)
    found = cv2.HoughLinesP(
        center_points,
        1,
        np.pi / 180,
        threshold=PROTECT_LINE_THRESHOLD,
        minLineLength=PROTECT_LINE_MIN_LENGTH,
        maxLineGap=PROTECT_LINE_MAX_GAP,
    )
    protected = np.zeros_like(raw_binary)
    if found is not None:
        for x1, y1, x2, y2 in np.asarray(found).reshape(-1, 4):
            cv2.line(
                protected,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                255,
                PROTECT_LINE_THICKNESS,
            )
    return cv2.bitwise_and(protected, center_roi)


def filter_navigation_points(
    points: np.ndarray,
    protected_lines: Optional[np.ndarray] = None,
) -> np.ndarray:
    if len(points) == 0:
        return points
    xy = polar_to_xy(points)
    inside_center_square = (
        (np.abs(xy[:, 0]) <= CENTER_EXCLUSION_HALF_SIZE_MM)
        & (np.abs(xy[:, 1]) <= CENTER_EXCLUSION_HALF_SIZE_MM)
    )
    protected = np.zeros(len(points), dtype=bool)
    if protected_lines is not None:
        center = SOURCE_SIZE // 2
        pixels_per_mm = SOURCE_SIZE / REAL_SIZE_MM
        px = np.rint(center + xy[:, 0] * pixels_per_mm).astype(np.int32)
        py = np.rint(center - xy[:, 1] * pixels_per_mm).astype(np.int32)
        visible = (
            (px >= 0) & (px < SOURCE_SIZE)
            & (py >= 0) & (py < SOURCE_SIZE)
        )
        protected[visible] = protected_lines[py[visible], px[visible]] > 0
    keep = ((~inside_center_square) | protected) & (points[:, 1] <= MAX_DISTANCE_MM)
    return points[keep]


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
    return [
        tuple(int(value) for value in line)
        for line in np.asarray(found).reshape(-1, 4)
    ]


def remove_scattered_components(binary: np.ndarray) -> np.ndarray:
    """PCAで短く丸い孤立成分を除去し、斜めを含む壁成分を残す。"""
    cleaned = np.zeros_like(binary)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for contour in contours:
        x, y, width, height = cv2.boundingRect(contour)
        shifted = contour - np.array([[[x, y]]], dtype=contour.dtype)
        component = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(component, [shifted], -1, 255, cv2.FILLED)
        ys, xs = np.nonzero(component)
        if len(xs) < 2:
            continue
        area = len(xs)
        coordinates = np.column_stack((xs, ys)).astype(np.float32)
        centered = coordinates - coordinates.mean(axis=0, keepdims=True)
        covariance = np.cov(centered, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        major_index = int(np.argmax(eigenvalues))
        major_value = max(float(eigenvalues[major_index]), 0.0)
        minor_value = max(float(eigenvalues[1 - major_index]), 0.0)
        linearity = math.sqrt((major_value + 1e-6) / (minor_value + 1e-6))
        projection = centered @ eigenvectors[:, major_index]
        principal_span = float(projection.max() - projection.min())
        wall_shaped = (
            area >= NOISE_MIN_AREA
            and principal_span >= NOISE_MIN_PRINCIPAL_SPAN
            and linearity >= NOISE_MIN_LINEARITY
        )
        large_component = (
            area >= NOISE_KEEP_LARGE_AREA
            and principal_span >= NOISE_MIN_PRINCIPAL_SPAN
        )
        if wall_shaped or large_component:
            target = cleaned[y:y + height, x:x + width]
            cv2.bitwise_or(target, component, dst=target)
    return cleaned


def process_pipeline(raw_points: np.ndarray) -> PipelineResult:
    """Raw点群からHough線分までを一度だけ計算する。"""
    raw_binary = points_to_binary(raw_points)
    protected_lines = protected_center_line_mask(raw_binary)
    filtered = filter_navigation_points(raw_points, protected_lines)
    filtered_binary = points_to_binary(filtered)
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE)
    )
    erode_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (ERODE_KERNEL_SIZE, ERODE_KERNEL_SIZE)
    )
    closed = cv2.morphologyEx(filtered_binary, cv2.MORPH_CLOSE, close_kernel)
    eroded = cv2.erode(closed, erode_kernel, iterations=1)
    cleaned = remove_scattered_components(eroded)
    lines = hough_lines(cleaned)
    return PipelineResult(
        raw_binary=raw_binary,
        filtered_binary=filtered_binary,
        closed=closed,
        eroded=eroded,
        cleaned=cleaned,
        lines=lines,
    )


def make_panel(image: np.ndarray, title: str) -> np.ndarray:
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image
    resized = cv2.resize(color, (PANEL_SIZE, PANEL_SIZE), interpolation=cv2.INTER_AREA)
    panel = np.full((PANEL_SIZE + PANEL_HEADER, PANEL_SIZE, 3), 18, dtype=np.uint8)
    panel[PANEL_HEADER:] = resized
    cv2.putText(panel, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (255, 255, 255), 2)
    cv2.rectangle(panel, (0, 0), (PANEL_SIZE - 1, PANEL_SIZE + PANEL_HEADER - 1), (110, 110, 110), 2)
    return panel


def create_dashboard_from_result(
    result: PipelineResult,
    annotations: Optional[list[str]] = None,
) -> np.ndarray:
    hough_image = cv2.cvtColor(result.cleaned, cv2.COLOR_GRAY2BGR)
    for x1, y1, x2, y2 in result.lines:
        cv2.line(hough_image, (x1, y1), (x2, y2), (0, 200, 255), 2)

    center = SOURCE_SIZE // 2
    radius_px = max(3, round(ROBOT_RADIUS_MM * SOURCE_SIZE / REAL_SIZE_MM))
    cv2.circle(hough_image, (center, center), radius_px, (0, 0, 255), 2)
    cv2.arrowedLine(hough_image, (center, center), (center, center - 45), (255, 255, 0), 2)
    cv2.putText(hough_image, f"lines: {len(result.lines)}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    for index, text in enumerate(annotations or []):
        cv2.putText(
            hough_image,
            text,
            (12, 56 + index * 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 0) if index < 3 else (255, 255, 0),
            2,
        )
    cv2.putText(hough_image, PROGRAM_VERSION, (12, SOURCE_SIZE - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)

    stages = [
        ("1. Raw (Unfiltered)", add_grid(result.raw_binary)),
        ("2. 20x20 Filter + Line Keep", add_grid(result.filtered_binary)),
        ("3. Mild Closing", result.closed),
        ("4. After Erosion", result.eroded),
        ("5. Noise Filter", result.cleaned),
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
    mm_per_pixel = REAL_SIZE_MM / SOURCE_SIZE
    corridor_px = FRONT_HALF_WIDTH_MM / mm_per_pixel
    robot_x = robot_y = SOURCE_SIZE // 2
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
        center_distance = (robot_y - y_at_robot) * mm_per_pixel
        clearance = center_distance - ROBOT_RADIUS_MM
        if clearance > 0:
            candidates.append(clearance)
    return min(candidates) if candidates else None


def side_wall_clearances_mm(
    lines: list[tuple[int, int, int, int]],
) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
    """左右壁の現在位置と15 cm前方での予測隙間を返す。"""
    mm_per_pixel = REAL_SIZE_MM / SOURCE_SIZE
    lookahead_px = SIDE_WALL_LOOKAHEAD_MM / mm_per_pixel
    guidance_ahead_px = SIDE_GUIDANCE_LOOKAHEAD_MM / mm_per_pixel
    robot_x = robot_y = SOURCE_SIZE // 2
    left_candidates: list[tuple[float, float]] = []
    right_candidates: list[tuple[float, float]] = []

    for x1, y1, x2, y2 in lines:
        dx, dy = x2 - x1, y2 - y1
        if math.hypot(dx, dy) < HOUGH_MIN_LINE_LENGTH or abs(dy) < 1:
            continue

        # 画像上で縦に近い線だけを左右壁として扱う。
        parallel_angle = math.degrees(math.atan2(dx, -dy))
        while parallel_angle > 90:
            parallel_angle -= 180
        while parallel_angle < -90:
            parallel_angle += 180
        if abs(parallel_angle) > SIDE_MAX_PARALLEL_ANGLE_DEG:
            continue

        # ロボット位置から前後25 cm以内に存在する線だけを採用する。
        top, bottom = sorted((y1, y2))
        if robot_y < top - lookahead_px or robot_y > bottom + lookahead_px:
            continue

        # 線をロボット中心Yまで延長し、ロボット側面から壁までを求める。
        x_at_robot = x1 + (robot_y - y1) * dx / dy
        ahead_y = robot_y - guidance_ahead_px
        x_at_ahead = x1 + (ahead_y - y1) * dx / dy
        clearance = abs(x_at_robot - robot_x) * mm_per_pixel - ROBOT_RADIUS_MM
        if clearance < 0 or clearance > SIDE_MAX_DETECTION_CLEARANCE_MM:
            continue
        if x_at_robot < robot_x:
            ahead_clearance = (robot_x - x_at_ahead) * mm_per_pixel - ROBOT_RADIUS_MM
            left_candidates.append((clearance, ahead_clearance))
        elif x_at_robot > robot_x:
            ahead_clearance = (x_at_ahead - robot_x) * mm_per_pixel - ROBOT_RADIUS_MM
            right_candidates.append((clearance, ahead_clearance))

    # 同じ壁から複数線が出ても外れ値へ引かれにくい中央値を使う。
    left = float(np.median([value[0] for value in left_candidates])) if left_candidates else None
    right = float(np.median([value[0] for value in right_candidates])) if right_candidates else None
    left_ahead = float(np.median([value[1] for value in left_candidates])) if left_candidates else None
    right_ahead = float(np.median([value[1] for value in right_candidates])) if right_candidates else None
    return left, right, left_ahead, right_ahead


def wall_following_errors(
    left_clearance: Optional[float],
    right_clearance: Optional[float],
    left_ahead: Optional[float],
    right_ahead: Optional[float],
) -> tuple[float, float, str]:
    """壁との横距離誤差(mm)と平行角度誤差(deg)を別々に返す。"""
    if left_clearance is not None and right_clearance is not None:
        distance_error = left_clearance - right_clearance
        mode = "TRACE CENTER"
    elif left_clearance is not None:
        distance_error = left_clearance - SIDE_TARGET_CLEARANCE_MM
        mode = "TRACE LEFT 10cm"
    elif right_clearance is not None:
        distance_error = SIDE_TARGET_CLEARANCE_MM - right_clearance
        mode = "TRACE RIGHT 10cm"
    else:
        return 0.0, 0.0, "OPEN"

    # 左壁は先で広がると正、右壁は先で狭まると正。
    # どちらも同じ旋回方向になる符号へ揃えてから中央値を使う。
    angle_candidates: list[float] = []
    if left_clearance is not None and left_ahead is not None:
        angle_candidates.append(math.degrees(math.atan2(
            left_ahead - left_clearance,
            SIDE_GUIDANCE_LOOKAHEAD_MM,
        )))
    if right_clearance is not None and right_ahead is not None:
        angle_candidates.append(math.degrees(math.atan2(
            right_clearance - right_ahead,
            SIDE_GUIDANCE_LOOKAHEAD_MM,
        )))
    angle_error = float(np.median(angle_candidates)) if angle_candidates else 0.0
    return distance_error, angle_error, mode


def process_and_render(
    scan: Scan,
    travelled_mm: float,
    command: str,
) -> tuple[
    bytes,
    Optional[float],
    Optional[float],
    Optional[float],
    Optional[float],
    Optional[float],
    Optional[float],
]:
    """共通パイプラインを一度だけ処理し、判断と6画面表示へ共有する。"""
    raw_points = scan.points[:, :2].astype(np.float32, copy=False)
    result = process_pipeline(raw_points)
    error = parallel_error_deg(result.lines)
    wall_clearance = front_wall_clearance_mm(result.lines)
    (
        left_clearance,
        right_clearance,
        left_ahead,
        right_ahead,
    ) = side_wall_clearances_mm(result.lines)
    wall_text = "---" if wall_clearance is None else f"{wall_clearance / 10:.1f} cm"
    error_text = "---" if error is None else f"{error:+.1f} deg"
    left_text = "---" if left_clearance is None else f"{left_clearance / 10:.1f}"
    right_text = "---" if right_clearance is None else f"{right_clearance / 10:.1f}"
    left_ahead_text = "---" if left_ahead is None else f"{left_ahead / 10:.1f}"
    right_ahead_text = "---" if right_ahead is None else f"{right_ahead / 10:.1f}"
    _, follow_angle, follow_mode = wall_following_errors(
        left_clearance,
        right_clearance,
        left_ahead,
        right_ahead,
    )
    dashboard = create_dashboard_from_result(
        result,
        annotations=[
            f"processed wall: {wall_text}",
            f"side now L/R: {left_text} / {right_text} cm",
            f"side +15cm L/R: {left_ahead_text} / {right_ahead_text} cm",
            f"wall trace: {follow_mode} {follow_angle:+.1f} deg",
            f"Hough line: {error_text}",
            f"travel: {travelled_mm / 10:.1f} cm",
            command,
        ],
    )
    ok, encoded = cv2.imencode(
        ".jpg",
        dashboard,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("JPEGエンコードに失敗しました")
    return (
        encoded.tobytes(),
        error,
        wall_clearance,
        left_clearance,
        right_clearance,
        left_ahead,
        right_ahead,
    )


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
html,body{margin:0;width:100%;min-height:100%;background:#111;color:#eee;text-align:center;font-family:sans-serif}
h3{height:5vh;line-height:5vh;margin:0;font-size:clamp(18px,2.2vw,30px)}
img{display:block;width:min(98vw,1400px);height:94vh;object-fit:contain;margin:0 auto}
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
            time.sleep(1 / WEB_FPS)

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
        left_clearance: Optional[float] = None
        right_clearance: Optional[float] = None
        left_ahead: Optional[float] = None
        right_ahead: Optional[float] = None
        pause_until: Optional[float] = None
        while not stopping.is_set():
            now = time.monotonic()
            if lidar.error:
                raise RuntimeError(f"LiDAR受信エラー: {lidar.error}")
            scan = lidar.snapshot()
            if now - scan.timestamp > LIDAR_STALE_SECONDS:
                raise RuntimeError("LiDARデータが停止しました")

            if now - last_render >= RENDER_INTERVAL_SECONDS:
                (
                    frame,
                    line_error,
                    wall_clearance,
                    left_clearance,
                    right_clearance,
                    left_ahead,
                    right_ahead,
                ) = process_and_render(scan, travelled, command)
                view.update(frame)
                last_render = now

            if args.view_only:
                time.sleep(0.01)
                continue
            assert motors is not None
            side_values = [
                value for value in (left_clearance, right_clearance)
                if value is not None
            ]
            if side_values and min(side_values) <= SIDE_EMERGENCY_CLEARANCE_MM:
                command = "STOP: SIDE WALL <= 2 cm"
                motors.stop()
                print(
                    "側面壁へ接近したため緊急停止: "
                    f"nearest={min(side_values) / 10:.1f} cm"
                )
                break
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
                    if args.segments > 0 and segment_number >= args.segments:
                        command = f"STOP: {segment_number} SEGMENT COMPLETE"
                        print(
                            f"指定区間完了: segments={segment_number}, "
                            f"total={total_travelled / 10:.1f} cm"
                        )
                        break
                    command = f"PAUSE: SEGMENT {segment_number}"
                    pause_until = now + SEGMENT_PAUSE_SECONDS
                    print(f"segment {segment_number} completed: {travelled / 10:.1f} cm, total={total_travelled / 10:.1f} cm")
                    last_control = now
                    continue
                current_base = NEAR_WALL_POWER if wall_clearance is not None and wall_clearance <= SLOWDOWN_CLEARANCE_MM else BASE_POWER
                distance_error, angle_error, guidance_mode = wall_following_errors(
                    left_clearance,
                    right_clearance,
                    left_ahead,
                    right_ahead,
                )
                # ライントレースと同様に、壁からの横ずれと壁に対する
                # 角度ずれを別々に戻す。片方が0でももう片方は補正を続ける。
                distance_correction = KP_WALL_DISTANCE * distance_error
                angle_correction = KP_WALL_ANGLE * angle_error
                correction = int(np.clip(
                    args.steering_sign * (distance_correction + angle_correction),
                    -MAX_STEERING,
                    MAX_STEERING,
                ))
                left, right = mix_motor_power(current_base, correction)
                motors.drive(left, right)
                command = f"{guidance_mode} S{segment_number} L={left} R={right}"
                wall_text = "---" if wall_clearance is None else f"{wall_clearance / 10:.1f}"
                side_left_text = "---" if left_clearance is None else f"{left_clearance / 10:.1f}"
                side_right_text = "---" if right_clearance is None else f"{right_clearance / 10:.1f}"
                ahead_left_text = "---" if left_ahead is None else f"{left_ahead / 10:.1f}"
                ahead_right_text = "---" if right_ahead is None else f"{right_ahead / 10:.1f}"
                print(
                    f"processed_wall={wall_text} cm "
                    f"side={side_left_text}/{side_right_text} cm "
                    f"ahead={ahead_left_text}/{ahead_right_text} cm "
                    f"mode={guidance_mode} travel={travelled / 10:5.1f} cm "
                    f"wall_angle={angle_error:+.1f} deg "
                    f"distance_error={distance_error:+.1f} mm "
                    f"correction={correction} L={left} R={right}"
                )
                last_control = now
            time.sleep(0.015)

        if not args.view_only:
            print("停止しました。Ctrl+Cで終了します。ブラウザ表示はそのまま確認できます。")
            while not stopping.is_set():
                scan = lidar.snapshot()
                frame, _, _, _, _, _, _ = process_and_render(scan, travelled, command)
                view.update(frame)
                time.sleep(RENDER_INTERVAL_SECONDS)
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
    parser.add_argument("--web-port", type=int, default=8000)
    parser.add_argument("--angle-offset", type=float, default=0.0)
    parser.add_argument(
        "--steering-sign",
        type=int,
        choices=(-1, 1),
        default=-1,
        help="補正方向。実機確認済みの既定値は-1、逆へ戻す場合は1",
    )
    parser.add_argument(
        "--segments",
        type=int,
        default=0,
        help="走行する30 cm区間数。1で30 cmを1回だけ、0で正面壁まで連続走行",
    )
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
