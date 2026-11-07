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

TARGET_CLEARANCE_MM = 100.0       # 正面壁との停止対象距離
STOP_COMMAND_CLEARANCE_MM = 100.0 # 処理後の正面Hough直線が10 cm以下で停止
SLOWDOWN_CLEARANCE_MM = 200.0
RUN_DISTANCE_MM = 320.0
FRONT_HALF_WIDTH_MM = 90.0
SIDE_TARGET_CLEARANCE_MM = 100.0
SIDE_MAX_DETECTION_CLEARANCE_MM = 250.0
SIDE_WALL_LOOKAHEAD_MM = 250.0
SIDE_GUIDANCE_LOOKAHEAD_MM = 150.0
SIDE_MAX_PARALLEL_ANGLE_DEG = 45.0
SIDE_FILTER_ALPHA = 0.70
SIDE_HOLD_SECONDS = 0.25
LIDAR_STALE_SECONDS = 0.5
POINT_LIFETIME_SECONDS = 0.35
RENDER_INTERVAL_SECONDS = 1 / WEB_FPS
CONTROL_INTERVAL_SECONDS = 0.05
SEGMENT_PAUSE_SECONDS = 0.5
PROGRAM_VERSION = "LiDAR continuous maze drive v5 230400 baud"
TURN_POWER = 700
TURN_FINE_POWER = 500
TURN_90_ENCODER_COUNTS = 2048
TURN_90_SCALE = 1.30  # 左右90度にも180度で良好だった倍率を初期値として適用
RIGHT_TURN_TRIM_DEG = 6.0  # 右90度だけ停止を6度早める実機調整
TURN_180_SCALE = 1.30  # 初回実測補正1.241から約5%増量
TURN_DECISION_SECONDS = 0.60
TURN_SETTLE_SECONDS = 0.45
TURN_TIMEOUT_SECONDS = 6.0
SIDE_WALL_VOTE_RATIO = 0.35
LIDAR_TURN_SEARCH_WINDOW_DEG = 18
LIDAR_TURN_TOLERANCE_DEG = 2.0
LIDAR_TURN_MIN_MATCHES = 24
LIDAR_TURN_MAX_SCORE = 0.22
LIDAR_TURN_FINE_ZONE_DEG = 15.0
TURN_ENCODER_SAFETY_RATIO = 1.15

BASE_POWER = 1200
NEAR_WALL_POWER = 750
ALIGN_BASE_POWER = 900
MIN_POWER = 700
MAX_POWER = 1200
KP_WALL_DISTANCE = 8.0
KP_WALL_ANGLE = 40.0
ALIGN_DISTANCE_THRESHOLD_MM = 15.0
ALIGN_ANGLE_THRESHOLD_DEG = 1.5
MAX_STEERING = 250

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


class SideWallFilter:
    """Hough線の瞬間的な欠落と距離の跳びを抑える短時間フィルター。"""

    def __init__(self) -> None:
        self.left: Optional[tuple[float, float, float]] = None
        self.right: Optional[tuple[float, float, float]] = None

    @staticmethod
    def _update_side(
        previous: Optional[tuple[float, float, float]],
        current: Optional[float],
        ahead: Optional[float],
        now: float,
    ) -> Optional[tuple[float, float, float]]:
        if current is not None and ahead is not None:
            if previous is None:
                return current, ahead, now
            old_current, old_ahead, _ = previous
            alpha = SIDE_FILTER_ALPHA
            return (
                alpha * current + (1.0 - alpha) * old_current,
                alpha * ahead + (1.0 - alpha) * old_ahead,
                now,
            )
        if previous is not None and now - previous[2] <= SIDE_HOLD_SECONDS:
            return previous
        return None

    def update(
        self,
        left: Optional[float],
        right: Optional[float],
        left_ahead: Optional[float],
        right_ahead: Optional[float],
        now: float,
    ) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        self.left = self._update_side(self.left, left, left_ahead, now)
        self.right = self._update_side(self.right, right, right_ahead, now)
        return (
            None if self.left is None else self.left[0],
            None if self.right is None else self.right[0],
            None if self.left is None else self.left[1],
            None if self.right is None else self.right[1],
        )


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


def check_lidar(lidar: MS200, scan: Scan) -> None:
    if lidar.error:
        raise RuntimeError(f"LiDAR受信エラー: {lidar.error}")
    if time.monotonic() - scan.timestamp > LIDAR_STALE_SECONDS:
        raise RuntimeError("LiDARデータが停止しました")


def render(
    lidar: MS200,
    view: SharedView,
    travelled_mm: float,
    command: str,
) -> tuple[
    Optional[float],
    Optional[float],
    Optional[float],
    Optional[float],
    Optional[float],
]:
    scan = lidar.snapshot()
    check_lidar(lidar, scan)
    (
        frame,
        _,
        front_clearance,
        left_clearance,
        right_clearance,
        left_ahead,
        right_ahead,
    ) = process_and_render(scan, travelled_mm, command)
    view.update(frame)
    return (
        front_clearance,
        left_clearance,
        right_clearance,
        left_ahead,
        right_ahead,
    )


def choose_turn_at_front_wall(
    lidar: MS200,
    view: SharedView,
    travelled_mm: float,
    stopping: threading.Event,
    decision_seconds: float,
) -> tuple[str, int, bool, bool]:
    """停止中だけ左右壁を複数回観測し、今回の旋回を一度だけ決める。"""
    deadline = time.monotonic() + decision_seconds
    frames = 0
    left_hits = 0
    right_hits = 0

    while not stopping.is_set() and time.monotonic() < deadline:
        _, left, right, _, _ = render(
            lidar,
            view,
            travelled_mm,
            "STOP: CHECK LEFT / RIGHT",
        )
        frames += 1
        left_hits += int(left is not None)
        right_hits += int(right is not None)
        time.sleep(RENDER_INTERVAL_SECONDS)

    required_hits = max(1, math.ceil(frames * SIDE_WALL_VOTE_RATIO))
    left_wall = left_hits >= required_hits
    right_wall = right_hits >= required_hits

    if left_wall and right_wall:
        return "LEFT", 180, left_wall, right_wall
    if not left_wall:
        # 左右とも開いている場合も、指定どおり左を優先する。
        return "LEFT", 90, left_wall, right_wall
    return "RIGHT", 90, left_wall, right_wall


def lidar_distance_profile(scan: Scan) -> np.ndarray:
    """スキャンを角度1度ごとの距離配列へ変換する。欠測はNaNのまま残す。"""
    profile = np.full(360, np.nan, dtype=np.float32)
    if scan.points.size == 0:
        return profile
    for angle_deg, distance_mm, _ in scan.points:
        if ROBOT_RADIUS_MM < distance_mm <= MAX_DISTANCE_MM:
            profile[int(round(float(angle_deg))) % 360] = float(distance_mm)
    return profile


def estimate_lidar_rotation_deg(
    reference_profile: np.ndarray,
    current_scan: Scan,
    expected_signed_deg: float,
) -> tuple[Optional[float], float, int]:
    """開始時と現在の距離波形を円環照合し、LiDARで旋回量を推定する。"""
    current_profile = lidar_distance_profile(current_scan)
    current_valid = np.isfinite(current_profile)
    center = int(round(expected_signed_deg))
    best_rotation: Optional[float] = None
    best_score = float("inf")
    best_matches = 0

    for rotation in range(
        center - LIDAR_TURN_SEARCH_WINDOW_DEG,
        center + LIDAR_TURN_SEARCH_WINDOW_DEG + 1,
    ):
        # 左へrotation度回ると、現在角aの物体は開始時角a+rotationにあった。
        shifted_reference = np.roll(reference_profile, -rotation)
        valid = current_valid & np.isfinite(shifted_reference)
        matches = int(np.count_nonzero(valid))
        if matches < LIDAR_TURN_MIN_MATCHES:
            continue

        reference_values = shifted_reference[valid]
        current_values = current_profile[valid]
        # 距離の絶対差ではなく比率の対数を使い、遠方点だけが支配しないようにする。
        errors = np.abs(np.log(
            np.maximum(current_values, 1.0) / np.maximum(reference_values, 1.0)
        ))
        score = float(np.median(errors))
        # 同程度の候補ではエンコーダー予測に近い方を選び、対称な部屋での飛びを防ぐ。
        score += abs(rotation - expected_signed_deg) * 0.001
        if score < best_score:
            best_rotation = float(rotation)
            best_score = score
            best_matches = matches

    if best_rotation is None or best_score > LIDAR_TURN_MAX_SCORE:
        return None, best_score, best_matches
    return best_rotation, best_score, best_matches


def turn_in_place(
    motors: Motors,
    lidar: MS200,
    view: SharedView,
    travelled_mm: float,
    stopping: threading.Event,
    direction: str,
    angle_deg: float,
    power: int,
    counts_per_90: float,
    timeout_seconds: float,
) -> None:
    """エンコーダーで範囲を絞り、LiDAR照合で目標角へ合わせて旋回する。"""
    target_counts = counts_per_90 * angle_deg / 90.0
    target_signed_deg = float(angle_deg if direction == "LEFT" else -angle_deg)
    print(
        f"旋回開始: direction={direction}, target={angle_deg} deg, "
        f"encoder_target={target_counts:.0f}, LiDAR tolerance={LIDAR_TURN_TOLERANCE_DEG:.1f} deg"
    )
    reference_scan = lidar.snapshot()
    check_lidar(lidar, reference_scan)
    reference_profile = lidar_distance_profile(reference_scan)
    previous_left, previous_right = motors.encoder_pair()
    accumulated_left = 0.0
    accumulated_right = 0.0
    started = time.monotonic()
    effective_timeout = timeout_seconds * angle_deg / 90.0
    last_render = 0.0
    lidar_rotation: Optional[float] = None
    lidar_score = float("inf")
    lidar_matches = 0
    finish_reason = "ENCODER SAFETY LIMIT"

    if direction == "LEFT":
        left_power, right_power = power, -power
    elif direction == "RIGHT":
        left_power, right_power = -power, power
    else:
        raise ValueError(f"不明な旋回方向: {direction}")

    command = f"TURN {direction} {angle_deg} deg"
    motors.drive(left_power, right_power, force=True)
    try:
        while not stopping.is_set():
            now = time.monotonic()
            if now - started > effective_timeout:
                raise RuntimeError(
                    f"{direction} {angle_deg}度旋回が"
                    f"{effective_timeout:.1f}秒以内に完了しません"
                )

            current_left, current_right = motors.encoder_pair()
            accumulated_left += abs(signed_encoder_delta(current_left, previous_left))
            accumulated_right += abs(signed_encoder_delta(current_right, previous_right))
            previous_left, previous_right = current_left, current_right
            average_counts = (accumulated_left + accumulated_right) / 2.0
            encoder_angle = average_counts / counts_per_90 * 90.0
            expected_signed_deg = encoder_angle if direction == "LEFT" else -encoder_angle

            scan = lidar.snapshot()
            check_lidar(lidar, scan)
            lidar_rotation, lidar_score, lidar_matches = estimate_lidar_rotation_deg(
                reference_profile,
                scan,
                expected_signed_deg,
            )
            lidar_remaining = (
                None
                if lidar_rotation is None
                else abs(target_signed_deg - lidar_rotation)
            )

            if lidar_remaining is not None and lidar_remaining <= LIDAR_TURN_FINE_ZONE_DEG:
                # 実機では旧減速開始位置で姿勢が合うため、ここで旋回を終了する。
                finish_reason = "LIDAR STOP AT FORMER SLOWDOWN"
                break

            if now - last_render >= RENDER_INTERVAL_SECONDS:
                lidar_text = (
                    "---"
                    if lidar_rotation is None
                    else f"{lidar_rotation:+.0f}deg"
                )
                frame, _, _, _, _, _, _ = process_and_render(
                    scan,
                    travelled_mm,
                    f"{command} "
                    f"ENC={encoder_angle:.0f} LIDAR={lidar_text}",
                )
                view.update(frame)
                last_render = now

            # LiDAR照合が成立しない環境では、較正済みエンコーダー値で停止する。
            if lidar_rotation is None and average_counts >= target_counts:
                finish_reason = "ENCODER FALLBACK"
                break

            # 照合不能や環境の対称性があっても回り続けないための最終安全限界。
            if average_counts >= target_counts * TURN_ENCODER_SAFETY_RATIO:
                break
            time.sleep(CONTROL_INTERVAL_SECONDS)
    finally:
        motors.stop(force=True)

    print(
        f"旋回完了: direction={direction}, angle={angle_deg}, "
        f"reason={finish_reason}, "
        f"encoder={((accumulated_left + accumulated_right) / 2.0):.0f}/{target_counts:.0f}, "
        f"lidar={lidar_rotation}, score={lidar_score:.3f}, matches={lidar_matches}"
    )


def wait_after_turn(
    lidar: MS200,
    view: SharedView,
    stopping: threading.Event,
    seconds: float,
) -> None:
    """旋回前の古い点群が消えるまで停止し、新しい向きの画面を更新する。"""
    deadline = time.monotonic() + seconds
    while not stopping.is_set() and time.monotonic() < deadline:
        render(lidar, view, 0.0, "TURN COMPLETE: LIDAR REFRESH")
        time.sleep(RENDER_INTERVAL_SECONDS)




def run(args: argparse.Namespace) -> None:
    lidar = MS200(args.lidar_port, args.lidar_baud, args.angle_offset)
    motors: Optional[Motors] = None
    view = SharedView()
    server = start_web(view, args.web_port)
    stopping = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopping.set())

    print(f"{PROGRAM_VERSION} started")
    print(f"LiDAR settings: port={args.lidar_port}, baud={args.lidar_baud}")
    print(f"Windowsで http://<Raspberry PiのIP>:{args.web_port}/ を開いてください")
    print(
        "進路規則: 両壁=180度、左開放=左90度、右開放=右90度、"
        "両側開放=左90度"
    )

    travelled = 0.0
    total_travelled = 0.0
    segment_number = 1
    command = "VIEW ONLY" if args.view_only else "WAIT"
    side_filter = SideWallFilter()

    try:
        lidar.start()
        wait_for_lidar(lidar)
        if not args.view_only:
            motors = Motors(args.motor_port)
            motors.start()
            print("最初は車輪を浮かせ、旋回方向を確認してください。")
            input("Enterを押すと連続走行を開始します: ")
            previous_left, previous_right = motors.encoder_pair()
            motors.drive(BASE_POWER, BASE_POWER)
            command = "FORWARD"

        last_control = 0.0
        last_render = 0.0
        wall_clearance: Optional[float] = None
        left_clearance: Optional[float] = None
        right_clearance: Optional[float] = None
        left_ahead: Optional[float] = None
        right_ahead: Optional[float] = None
        pause_until: Optional[float] = None

        while not stopping.is_set():
            now = time.monotonic()
            scan = lidar.snapshot()
            check_lidar(lidar, scan)

            if now - last_render >= RENDER_INTERVAL_SECONDS:
                (
                    frame,
                    _,
                    wall_clearance,
                    raw_left_clearance,
                    raw_right_clearance,
                    raw_left_ahead,
                    raw_right_ahead,
                ) = process_and_render(scan, travelled, command)
                (
                    left_clearance,
                    right_clearance,
                    left_ahead,
                    right_ahead,
                ) = side_filter.update(
                    raw_left_clearance,
                    raw_right_clearance,
                    raw_left_ahead,
                    raw_right_ahead,
                    now,
                )
                view.update(frame)
                last_render = now

            if args.view_only:
                time.sleep(0.01)
                continue
            assert motors is not None

            # 30 cm区切りによる停止では旋回判断を行わない。
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
                wall_clearance = None
                continue

            # 前方壁で止まった場合だけ、停止中の左右観測から旋回を決定する。
            stop_command_mm = args.stop_command_cm * 10.0
            if wall_clearance is not None and wall_clearance <= stop_command_mm:
                motors.stop(force=True)
                command = "STOP: FRONT WALL - DECIDE TURN"
                print(f"正面壁で停止: front={wall_clearance / 10:.1f} cm")

                direction, angle_deg, left_wall, right_wall = choose_turn_at_front_wall(
                    lidar,
                    view,
                    travelled,
                    stopping,
                    args.turn_decision_seconds,
                )
                if stopping.is_set():
                    break
                print(
                    f"区切りで進路決定: left_wall={left_wall}, "
                    f"right_wall={right_wall} -> {direction} {angle_deg}度"
                )
                turn_in_place(
                    motors,
                    lidar,
                    view,
                    travelled,
                    stopping,
                    direction,
                    angle_deg - (RIGHT_TURN_TRIM_DEG if direction == "RIGHT" and angle_deg == 90 else 0.0),
                    args.turn_power,
                    args.turn_90_counts
                    * (args.turn_180_scale if angle_deg == 180 else args.turn_90_scale),
                    args.turn_timeout_seconds,
                )
                if stopping.is_set():
                    break
                wait_after_turn(
                    lidar,
                    view,
                    stopping,
                    args.turn_settle_seconds,
                )
                if stopping.is_set():
                    break

                # 新しい向きでは、旋回前の左右壁履歴と走行区間を引き継がない。
                side_filter = SideWallFilter()
                wall_clearance = None
                left_clearance = None
                right_clearance = None
                left_ahead = None
                right_ahead = None
                travelled = 0.0
                segment_number += 1
                previous_left, previous_right = motors.encoder_pair()
                motors.drive(BASE_POWER, BASE_POWER)
                command = f"FORWARD AFTER TURN S{segment_number}"
                last_control = time.monotonic()
                last_render = 0.0
                continue

            if now - last_control >= CONTROL_INTERVAL_SECONDS:
                current_left, current_right = motors.encoder_pair()
                dl = abs(signed_encoder_delta(current_left, previous_left))
                dr = abs(signed_encoder_delta(current_right, previous_right))
                previous_left, previous_right = current_left, current_right
                step_mm = ((dl + dr) / 2.0) * (
                    math.pi * WHEEL_DIAMETER_MM / ENCODER_COUNTS
                )
                travelled += step_mm
                total_travelled += step_mm

                if travelled >= RUN_DISTANCE_MM:
                    motors.stop(force=True)
                    command = f"PAUSE: SEGMENT {segment_number}"
                    pause_until = now + SEGMENT_PAUSE_SECONDS
                    print(
                        f"segment {segment_number} completed without turn: "
                        f"{travelled / 10:.1f} cm, total={total_travelled / 10:.1f} cm"
                    )
                    last_control = now
                    continue

                distance_error, angle_error, guidance_mode = wall_following_errors(
                    left_clearance,
                    right_clearance,
                    left_ahead,
                    right_ahead,
                )
                current_base = (
                    NEAR_WALL_POWER
                    if wall_clearance is not None
                    and wall_clearance <= SLOWDOWN_CLEARANCE_MM
                    else BASE_POWER
                )
                if (
                    abs(distance_error) >= ALIGN_DISTANCE_THRESHOLD_MM
                    or abs(angle_error) >= ALIGN_ANGLE_THRESHOLD_DEG
                ):
                    current_base = min(current_base, ALIGN_BASE_POWER)
                correction = int(np.clip(
                    args.steering_sign
                    * (
                        KP_WALL_DISTANCE * distance_error
                        + KP_WALL_ANGLE * angle_error
                    ),
                    -MAX_STEERING,
                    MAX_STEERING,
                ))
                left_power, right_power = mix_motor_power(current_base, correction)
                motors.drive(left_power, right_power)
                command = (
                    f"{guidance_mode} S{segment_number} "
                    f"L={left_power} R={right_power}"
                )
                front_text = "---" if wall_clearance is None else f"{wall_clearance / 10:.1f}"
                left_text = "---" if left_clearance is None else f"{left_clearance / 10:.1f}"
                right_text = "---" if right_clearance is None else f"{right_clearance / 10:.1f}"
                print(
                    f"front={front_text} cm side={left_text}/{right_text} cm "
                    f"mode={guidance_mode} travel={travelled / 10:.1f} cm "
                    f"correction={correction} L={left_power} R={right_power}"
                )
                last_control = now

            time.sleep(0.015)
    finally:
        if motors is not None:
            motors.close()
        lidar.stop()
        server.should_exit = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="正面壁で進路を決め、低速旋回しながら連続走行する"
    )
    parser.add_argument("--lidar-port", default="/dev/ttyAMA0")
    parser.add_argument("--lidar-baud", type=int, default=230400)
    parser.add_argument("--motor-port", default="/dev/ttyUSB0")
    parser.add_argument("--angle-offset", type=float, default=0.0)
    parser.add_argument("--web-port", type=int, default=8000)
    parser.add_argument("--view-only", action="store_true")
    parser.add_argument("--steering-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument(
        "--stop-command-cm",
        type=float,
        default=STOP_COMMAND_CLEARANCE_MM / 10.0,
        help="正面壁の停止・旋回判断距離（既定10 cm）",
    )
    parser.add_argument(
        "--turn-power",
        type=int,
        default=TURN_POWER,
        help="旋回出力。前進の既定1200より小さい値にする（既定700）",
    )
    parser.add_argument(
        "--turn-90-counts",
        type=int,
        default=TURN_90_ENCODER_COUNTS,
        help="90度旋回に必要な平均エンコーダーカウント（既定2048）",
    )
    parser.add_argument(
        "--turn-90-scale",
        type=float,
        default=TURN_90_SCALE,
        help="左右90度旋回のエンコーダー換算倍率（既定1.30）",
    )
    parser.add_argument(
        "--turn-180-scale",
        type=float,
        default=TURN_180_SCALE,
        help="180度旋回だけのエンコーダー換算倍率（既定1.30）",
    )
    parser.add_argument(
        "--turn-decision-seconds",
        type=float,
        default=TURN_DECISION_SECONDS,
        help="正面停止後に左右壁を観測する時間（既定0.60秒）",
    )
    parser.add_argument(
        "--turn-settle-seconds",
        type=float,
        default=TURN_SETTLE_SECONDS,
        help="旋回後に古い点群が消えるまで待つ時間（既定0.45秒）",
    )
    parser.add_argument(
        "--turn-timeout-seconds",
        type=float,
        default=TURN_TIMEOUT_SECONDS,
        help="旋回異常時の停止タイムアウト（既定6秒）",
    )
    args = parser.parse_args()
    if not 0 < args.turn_power < BASE_POWER:
        parser.error(f"--turn-power は1以上{BASE_POWER - 1}以下にしてください")
    if args.turn_90_counts <= 0:
        parser.error("--turn-90-counts は正の値にしてください")
    if not math.isfinite(args.turn_90_scale) or args.turn_90_scale <= 0:
        parser.error("--turn-90-scale は有限の正の値にしてください")
    if not math.isfinite(args.turn_180_scale) or args.turn_180_scale <= 0:
        parser.error("--turn-180-scale は有限の正の値にしてください")
    if args.turn_decision_seconds <= 0 or args.turn_settle_seconds < 0:
        parser.error("観測時間は正、整定時間は0以上にしてください")
    return args


if __name__ == "__main__":
    try:
        run(parse_args())
    except KeyboardInterrupt:
        pass




