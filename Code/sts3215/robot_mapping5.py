#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS200による32cm区間走行・旋回・タイル地図・Web表示を内蔵する。"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
import os
import errno
import fcntl
import socket
from pathlib import Path
import signal
import sys
import select
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

# Import the currently deployed PNG mirror module from project root.
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))
from Display_Process.Log.Logger5 import save_display_mirror, LCD_MIRROR_PATH


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

# 2026-11-01ユーザー指示：正常動作した基準版の壁目標・距離換算は変更しない。
TARGET_CLEARANCE_MM = 100.0       # 正面壁との停止対象距離
STOP_COMMAND_CLEARANCE_MM = 100.0 # 処理後の正面Hough直線が10 cm以下で停止
SLOWDOWN_CLEARANCE_MM = 200.0
RUN_DISTANCE_MM = 320.0
MAP_MIN_ADVANCE_MM = 50.0  # 前進距離5cm以上で停止したときだけ現在タイルを全置換
TILE_WALL_MARGIN_MM = 30.0  # 現タイル境界の壁判定余裕。次タイル奥の壁と区別する
FRONT_HALF_WIDTH_MM = 90.0
FRONT_MAX_WALL_ANGLE_DEG = 60.0
FRONT_NEAR_MAX_WALL_ANGLE_DEG = 85.0  # 旋回距離内では大きく斜めの前壁も対象
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
PROGRAM_VERSION = "MS200 baseline / FPV tile colors at 10Hz"
DEFAULT_COLOR_CAMERA = "/dev/video0"  # 2026-11-14実機確認済みUSBカメラ（映像入力）
COLOR_VIEW_FORWARD_MM = 100.0  # LiDAR原点から見たカメラ視野の代表点
COLOR_CAPTURE_INTERVAL_SECONDS = 0.1  # 色判定10Hz
CAMERA_PREVIEW_FPS = 30  # Web用ライブ表示。実機で調整可能
FRONT_APPROACH_EXTRA_MM = 50.0  # 停止目標は固定。タイル末の近い前壁へ最大5cm接近。
# 原点固定の報告を受け、全距離の移動量による座標更新・追加前進は既定で無効。
ENABLE_EXPERIMENTAL_BUMP_RECOVERY = False
DISTANT_GUIDANCE_INTERVAL_SECONDS = 0.2
CENTER_RETRY_TOLERANCE_MM = 50.0
CENTER_RETRY_MAX = 3
TURN_POWER = 700
ALIGNMENT_POWER = 700  # 実機の通常旋回と同じ始動可能な出力
TURN_FINE_POWER = 500
TURN_90_ENCODER_COUNTS = 2048
TURN_90_SCALE = 1.30  # 左右90度にも180度で良好だった倍率を初期値として適用
RIGHT_TURN_TRIM_DEG = 9.0  # 右90度だけ停止を9度早める実機調整
TURN_180_SCALE = 1.30  # 初回実測補正1.241から約5%増量
TURN_DECISION_SECONDS = 0.60
TURN_SETTLE_SECONDS = 0.45
TURN_TIMEOUT_SECONDS = 6.0
SIDE_WALL_VOTE_RATIO = 0.35
LIDAR_TURN_SEARCH_WINDOW_DEG = 18
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

# タイル移動終了後、通常の画像切り出し(前後500mm)や2500mm上限とは
# 独立した「MS200生点群の全有効距離」を使った側壁平行補正。
TILE_PARALLEL_TOLERANCE_DEG = 12.0  # 小角度の再旋回による過補正を避ける
TURN_PARALLEL_TOLERANCE_DEG = 20.0  # 通常旋回後は±20度以内を許容する
INITIAL_HEADING_TOLERANCE_DEG = 20.0
TILE_PARALLEL_MAX_CORRECTION_DEG = 12.0
TILE_PARALLEL_MAX_MEASURED_DEG = 20.0
TURN_PARALLEL_MAX_MEASURED_DEG = 30.0  # 旋回不足は側壁推定の有効角度内まで補正する
TILE_PARALLEL_MIN_SPAN_MM = 200.0
TILE_PARALLEL_INLIER_MM = 35.0
TILE_PARALLEL_MIN_INLIERS = 7
TILE_PARALLEL_RETRIES = 3
# 旋回後の補正は1回の観測では行わない（誤検出による過旋回を防止）。
TURN_ALIGN_TOLERANCE_DEG = 12.0
TURN_ALIGN_MEASUREMENT_SPREAD_DEG = 3.0
TURN_ALIGN_MAX_TRUSTED_ERROR_DEG = 22.0
TURN_ALIGN_MAX_STEP_DEG = 12.0
TURN_ALIGN_MAX_PASSES = 3
# 長距離点群から対向2壁を認識できる場合だけ中心位置を推定。
CENTERING_TOLERANCE_MM = 18.0
CENTERING_MAX_ADJUST_MM = 70.0
CENTERING_WALL_PAIR_TOLERANCE_MM = 55.0
CENTERING_LATERAL_MAX_STEER_MM = 65.0
# マネージャーと共有するリトライ通知。GPIO27緊急停止後だけ更新。
RETRY_SIGNAL_PATH = Path(os.environ.get(
    "ROBOT_RETRY_SIGNAL", "/dev/shm/robocup_lcd_retry.signal"))

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
    """4輪サーボ。緊急停止要求後の非ゼロ速度指令はすべて破棄する。"""

    def __init__(self, port: str, emergency: Optional[threading.Event] = None):
        self.servo = STS3215(port=port, baudrate=1_000_000, timeout=0.05)
        self._last_command: Optional[tuple[int, int]] = None
        self._emergency = emergency if emergency is not None else threading.Event()
        self._io_lock = threading.RLock()
        self._closed = False
        self._opened = False

    def start(self) -> None:
        with self._io_lock:
            if self._emergency.is_set():
                raise RuntimeError("緊急停止中のためモーターを初期化しません")
            self.servo.open()
            self._opened = True
            for motor_id in MOTOR_IDS:
                if self._emergency.is_set():
                    raise RuntimeError("モーター初期化中に緊急停止しました")
                if not self.servo.ping(motor_id):
                    raise RuntimeError(f"モーターID={motor_id}が応答しません")
            for motor_id in MOTOR_IDS:
                if self._emergency.is_set():
                    raise RuntimeError("モーター初期化中に緊急停止しました")
                self.servo.set_mode(motor_id, self.servo.MODE_VELOCITY)
                self.servo.torque(motor_id, True)
            self._send_locked(0, 0, force=True)

    @staticmethod
    def _raw_speed(motor_id: int, speed: int) -> int:
        signed = int(speed) * MOTOR_DIRECTION[motor_id]
        return abs(signed) if signed >= 0 else abs(signed) | 0x8000

    def _send_locked(self, left: int, right: int, force: bool = False) -> None:
        command = (int(left), int(right))
        if not force and command == self._last_command:
            return
        # この区間の4回の書き込みは _io_lock で排他する。
        for motor_id in LEFT_IDS:
            self.servo.write_u16(motor_id, self.servo.ADDR_GOAL_SPEED,
                                 self._raw_speed(motor_id, left))
        for motor_id in RIGHT_IDS:
            self.servo.write_u16(motor_id, self.servo.ADDR_GOAL_SPEED,
                                 self._raw_speed(motor_id, right))
        self._last_command = command

    def drive(self, left: int, right: int, force: bool = False) -> None:
        # GPIOコールバックが優先して立てるラッチは、ロックの前後で調べる。
        if self._emergency.is_set() and (left != 0 or right != 0):
            return
        with self._io_lock:
            if self._closed:
                return
            if self._emergency.is_set() and (left != 0 or right != 0):
                return
            self._send_locked(left, right, force=force)

    def stop(self, force: bool = False) -> None:
        self.drive(0, 0, force=force)

    def emergency_stop(self) -> None:
        """最優先で再加速禁止→全輪速度0。通信の完了までは物理停止を保証できない。"""
        self._emergency.set()
        with self._io_lock:
            if self._opened and not self._closed:
                self._send_locked(0, 0, force=True)

    def encoder_pair(self) -> tuple[int, int]:
        with self._io_lock:
            if self._emergency.is_set():
                raise RuntimeError("緊急停止によってエンコーダ読み取りを中断しました")
            left = self.servo.read_u16(1, self.servo.ADDR_PRESENT_POSITION)
            right = self.servo.read_u16(2, self.servo.ADDR_PRESENT_POSITION)
        return left, right

    def close(self) -> None:
        with self._io_lock:
            if self._closed:
                return
            self._closed = True
            if not self._opened:
                return
            try:
                self._send_locked(0, 0, force=True)
            finally:
                try:
                    for motor_id in MOTOR_IDS:
                        self.servo.torque(motor_id, False)
                finally:
                    self.servo.close()
                    self._opened = False


@dataclass(frozen=True)
class PipelineResult:
    """走行判断とWeb表示で共有する各画像処理工程。"""

    raw_binary: np.ndarray
    filtered_binary: np.ndarray
    closed: np.ndarray
    eroded: np.ndarray
    cleaned: np.ndarray
    lines: list[tuple[int, int, int, int]]


@dataclass(frozen=True)
class HoughScan:
    """前処理を通過し、Houghで抽出された線分だけを保持する。"""

    lines: np.ndarray
    timestamp: float


def capture_hough_scan(scan) -> HoughScan:
    if isinstance(scan, HoughScan):
        return scan
    result = process_pipeline(scan.points[:, :2].astype(np.float32, copy=False))
    return HoughScan(np.asarray(result.lines, dtype=np.float32).reshape(-1, 4), scan.timestamp)


def rotate_hough_scan(scan: HoughScan, degrees: float) -> HoughScan:
    """車体の左旋回を正とする。固定壁は画像では時計回りに移動する。"""
    radians = math.radians(degrees)
    rotation = np.array([[math.cos(radians), -math.sin(radians)],
                         [math.sin(radians), math.cos(radians)]], dtype=np.float32)
    center = float(SOURCE_SIZE // 2)
    endpoints = scan.lines.reshape(-1, 2)
    transformed = (endpoints - center) @ rotation.T + center
    return HoughScan(transformed.reshape(-1, 4), scan.timestamp)


def hough_line_image(scan: HoughScan) -> np.ndarray:
    image = np.zeros((SOURCE_SIZE, SOURCE_SIZE), dtype=np.uint8)
    for x1, y1, x2, y2 in scan.lines:
        cv2.line(image, (round(float(x1)), round(float(y1))),
                 (round(float(x2)), round(float(y2))), 255, 1)
    return image


def sample_hough_lines(scan: HoughScan) -> np.ndarray:
    samples = []
    for x1, y1, x2, y2 in scan.lines:
        length = math.hypot(x2 - x1, y2 - y1)
        count = max(2, min(120, math.ceil(length / 3.0)))
        ratio = np.linspace(0.0, 1.0, count)[:, None]
        samples.append(np.array([x1, y1]) + ratio * np.array([x2 - x1, y2 - y1]))
    if not samples:
        return np.empty((0, 2), dtype=np.float32)
    points = np.vstack(samples)
    visible = np.all((points >= 0) & (points < SOURCE_SIZE), axis=1)
    return points[visible][::max(1, math.ceil(int(visible.sum()) / 1000))]


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


def front_wall_measurement(
    lines, turn_clearance_mm: float = STOP_COMMAND_CLEARANCE_MM,
) -> tuple[Optional[float], Optional[float]]:
    """進路を横切る前壁の隙間と水平からの傾き。側壁の無制限な延長は使わない。"""
    candidates: list[tuple[float, float]] = []
    mm_per_pixel = REAL_SIZE_MM / SOURCE_SIZE
    corridor_px = FRONT_HALF_WIDTH_MM / mm_per_pixel
    robot_x = robot_y = SOURCE_SIZE // 2
    for x1, y1, x2, y2 in lines:
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 35 or abs(dx) < 1:
            continue
        # 斜めの前壁も対象にするが、ほぼ進路に平行な線は除外する。
        wall_angle = math.degrees(math.atan2(dy, dx))
        while wall_angle > 90:
            wall_angle -= 180
        while wall_angle < -90:
            wall_angle += 180
        if abs(wall_angle) > FRONT_NEAR_MAX_WALL_ANGLE_DEG:
            continue
        left, right = sorted((x1, x2))
        if right < robot_x - corridor_px or left > robot_x + corridor_px:
            continue
        overlap = min(right, robot_x + corridor_px) - max(left, robot_x - corridor_px)
        if overlap * mm_per_pixel < 20.0:
            continue
        outside = max(left - robot_x, robot_x - right, 0.0) * mm_per_pixel
        if outside > 40.0:
            continue
        # 直線をロボット中心Xまで延長し、前方距離を求める。
        y_at_robot = y1 + (robot_x - x1) * dy / dx
        center_distance = (robot_y - y_at_robot) * mm_per_pixel
        clearance = center_distance - ROBOT_RADIUS_MM
        if abs(wall_angle) > FRONT_MAX_WALL_ANGLE_DEG:
            # 近い斜め前壁は正対を待たず旋回対象にする。ただし中心線を
            # 実際に横切る線分に限り、側壁の延長を前壁と誤認しない。
            if clearance > turn_clearance_mm or not left <= robot_x <= right:
                continue
        if clearance > 0:
            candidates.append((clearance, wall_angle))
    return min(candidates) if candidates else (None, None)


def front_wall_clearance_mm(lines: list[tuple[int, int, int, int]]) -> Optional[float]:
    return front_wall_measurement(lines)[0]


def planned_wall_turn(direction: str, angle_deg: int, before_error: float,
                      wall_angle: Optional[float]) -> tuple[float, float, float]:
    """前壁に正対するための角度を通常旋回へ一度だけ加える。左旋回を正とする。"""
    signed_nominal = float(angle_deg if direction == "LEFT" else -angle_deg)
    correction = before_error if wall_angle is None else -wall_angle
    expected_rotation = signed_nominal + correction
    magnitude = abs(expected_rotation)
    if direction == "RIGHT" and angle_deg == 90:
        magnitude = max(5.0, magnitude - RIGHT_TURN_TRIM_DEG)
    return magnitude, expected_rotation, correction


def front_wall_turn_plan(lines, direction: str, nominal: int,
                         stop_command_mm: float) -> tuple[float, float, float]:
    """旋回直前の処理済み前壁角だけを通常旋回へ合成。測定不可なら元の角度。"""
    clearance, angle = front_wall_measurement(lines, stop_command_mm)
    if (clearance is None or angle is None or not math.isfinite(clearance)
            or not math.isfinite(angle)):
        angle = None
    plan = planned_wall_turn(direction, nominal, 0.0, angle)
    print(f"[TURN PLAN] front={clearance}, wall_angle={angle}, "
          f"add={plan[2]:+.1f}deg, expected={plan[1]:+.1f}deg, motor={plan[0]:.1f}deg")
    return plan


class FrontWallApproach:
    """通常距離に到達したときだけ、停止目標+5cm内の前壁へ接近を継続する。"""

    def __init__(self):
        self.limit_mm: Optional[float] = None

    def distance_stop(self, travelled_mm: float, target_mm: float,
                      clearance_mm: Optional[float], stop_mm: float) -> bool:
        if travelled_mm < target_mm:
            return False
        if (clearance_mm is None or not math.isfinite(clearance_mm)
                or not stop_mm < clearance_mm <= stop_mm + FRONT_APPROACH_EXTRA_MM):
            return True
        if self.limit_mm is None:
            self.limit_mm = travelled_mm + FRONT_APPROACH_EXTRA_MM
            print(f"[FRONT APPROACH] front={clearance_mm:.1f}mm, stop={stop_mm:.1f}mm, "
                  f"limit={self.limit_mm:.1f}mm")
        return travelled_mm >= self.limit_mm


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
        front_distance, _ = front_wall_measurement([(x1, y1, x2, y2)])
        if front_distance is not None and front_distance <= SLOWDOWN_CLEARANCE_MM:
            # 近くの斜め前壁を左右の壁として二重計上しない。
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
    # 距離目標に使うのは1タイル以内の側壁だけ。遠い壁へ寄せない。
    if left_clearance is not None and left_clearance > RUN_DISTANCE_MM:
        left_clearance = left_ahead = None
    if right_clearance is not None and right_clearance > RUN_DISTANCE_MM:
        right_clearance = right_ahead = None
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


def running_wall_guidance(scan: Scan, left, right, left_ahead, right_ahead):
    """近壁は従来どおり。近壁なしでは最も近い片壁の角度だけを使う。"""
    if any(d is not None and d <= RUN_DISTANCE_MM for d in (left, right)):
        return wall_following_errors(left, right, left_ahead, right_ahead)
    _, detail = measure_sidewall_parallel_unlimited(scan)
    sides = detail.get("sides", {})
    candidates = []
    for name in ("left", "right"):
        side = sides.get(name)
        if not isinstance(side, dict):
            continue
        clearance, angle = side.get("clearance_mm"), side.get("angle_deg")
        if (clearance is not None and angle is not None
                and math.isfinite(clearance) and clearance >= 0
                and math.isfinite(angle)):
            candidates.append((float(clearance), name, float(angle)))
    if not candidates:
        return 0.0, 0.0, "ANGLE ONLY: WALL UNCONFIRMED"
    # 左右角度の平均・距離差は使わない。距離は参照する片壁の選択だけに使う。
    # 全距離推定の延長切片が近くても、距離追従モードへ戻さない。
    clearance, name, angle = min(candidates, key=lambda value: value[0])
    # 生点のx=m*y+bと、既存の前方距離差から求める走行角度は符号が逆。
    return 0.0, -angle, f"ANGLE ONLY NEAREST {name.upper()} {clearance / 10:.1f}cm"


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


class ObservedWalls(dict):
    """新規辺に使う実測値と、旧地図の訂正を許可する2回一致辺を分離する。"""
    def __init__(self, values: dict, confirmed_edges=()):
        super().__init__(values)
        self.confirmed_edges = frozenset(confirmed_edges)


class TileMap:
    """座標はタイル単位。N=0,E=1,S=2,W=3で論理方位を管理する。"""

    DIRECTIONS = ("N", "E", "S", "W")
    STEPS = ((0, 1), (1, 0), (0, -1), (-1, 0))
    OPPOSITE = {"N": "S", "E": "W", "S": "N", "W": "E"}
    DELTAS = dict(zip(DIRECTIONS, STEPS))

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.tiles: dict[tuple[int, int], dict] = {}
        self.pending_camera_colors: dict[tuple[int, int], list[str]] = {}
        self.visited: set[tuple[int, int]] = set()
        # 走行によって実際に通過した隣接タイルの接続を保持する。
        # LiDARの壁スナップショットとは別の「走行実績」なので、
        # 既存タイルの観測値を全上書きしても帰り道の接続が失われない。
        self.confirmed_passages: set[frozenset[tuple[int, int]]] = set()
        self.x = self.y = self.heading = 0
        self.heading_angle_deg = 0.0
        self.path: Optional[Path] = None
        self.partial_mm = 0.0
        self.position_mm = [0.0, 0.0]
        self.last_motion_source = "START"
        self.heading_confirmation = None
        self.heading_status = "CONFIRMED"
        # 停止時のみ区間の移動を座標に反映する。走行中はマップを更新しない。
        self._segment_origin: Optional[tuple[float, float]] = None
        self._segment_entered_tiles: set[tuple[int, int]] = set()
        # GPIO27非常停止中〜Button1再開までは既存マップを完全に凍結。
        # 再初期化を理由にタイル・方位・保存JSONを更新しない。
        self._suspended = False

    def suspend_updates(self) -> None:
        """マップの内容を保持したまま更新を止め、未完了の走行区間を破棄。"""
        with self.lock:
            self._suspended = True
            self._segment_origin = None
            self._segment_entered_tiles.clear()

    def resume_updates(self) -> None:
        """Button1による新規走行を開始する際に限り更新を再許可。"""
        with self.lock:
            self._suspended = False

    @classmethod
    def _blank_tile(cls, x: int, y: int) -> dict:
        """新しい観測用タイル。古い属性や壁のknownを絶対に引き継がない。"""
        return {
            "x": x, "y": y, "A": 0, "B": 0, "C": 0, "D": 0,
            "E": 0, "W": 0, "S": 0, "N": 0,
            "known": {direction: False for direction in cls.DIRECTIONS},
            "visited": False,
        }

    def _tile(self, x: int, y: int) -> dict:
        tile = self.tiles.setdefault((x, y), self._blank_tile(x, y))
        pending = self.pending_camera_colors.pop((x, y), None)
        if pending is not None:
            tile.setdefault("camera_colors", pending)
        return tile

    def _wall(self, x: int, y: int, direction: str, value: int) -> None:
        tile = self._tile(x, y)
        tile[direction] = value
        tile["known"][direction] = True
        dx, dy = self.DELTAS[direction]
        neighbor = self.tiles.get((x + dx, y + dy))
        if neighbor is not None:
            opposite = self.OPPOSITE[direction]
            neighbor[opposite] = value
            neighbor["known"][opposite] = True

    def _replace_tile(self, x: int, y: int, absolute_walls: dict[str, Optional[int]],
                      attributes: Optional[dict] = None) -> None:
        """属性を新規作成し、壁は有効観測だけ更新。未検出で既知の辺を消さない。"""
        previous = self.tiles.get((x, y))
        tile = self._blank_tile(x, y)
        if previous is not None and "camera_colors" in previous:
            tile["camera_colors"] = list(previous["camera_colors"])
        elif (x, y) in self.pending_camera_colors:
            tile["camera_colors"] = self.pending_camera_colors.pop((x, y))
        if attributes is not None:
            for key in ("A", "B", "C", "D"):
                if key in attributes:
                    tile[key] = int(attributes[key])
        for direction in self.DIRECTIONS:
            value = absolute_walls.get(direction)
            dx, dy = self.DELTAS[direction]
            neighbor = (x + dx, y + dy)
            # 「線が見えない」は過去の壁なし/壁ありを否定する観測ではない。
            if value is None and previous is not None and previous["known"].get(direction, False):
                value = previous[direction]
            opposite = self.OPPOSITE[direction]
            adjacent = self.tiles.get(neighbor)
            if value is None and adjacent is not None and adjacent["known"].get(opposite, False):
                value = adjacent[opposite]
            if value is None and frozenset(((x, y), neighbor)) in self.confirmed_passages:
                value = 0
            if value is not None:
                tile[direction] = int(value)
                tile["known"][direction] = True
        # タイルを停止観測した時だけ到達済みにする。
        tile["visited"] = (x, y) in self.visited or (x, y) == (self.x, self.y)
        self.tiles[(x, y)] = tile
        for direction in self.DIRECTIONS:
            dx, dy = self.DELTAS[direction]
            neighbor = self.tiles.get((x + dx, y + dy))
            if neighbor is not None:
                opposite = self.OPPOSITE[direction]
                # 共有辺の有効観測を同じ値に揃える。
                neighbor[opposite] = tile[direction]
                neighbor["known"][opposite] = tile["known"][direction]

    def refresh_wall_observation(self, relative_walls: dict) -> None:
        """停止再観測で壁だけ更新。座標・到達済み・属性・移動量は変えない。"""
        with self.lock:
            if self._suspended or (self.x, self.y) not in self.visited:
                return
            changed = False
            tile = self._tile(self.x, self.y)
            for relative, value in relative_walls.items():
                if relative not in (0, 1, 2, 3) or value not in (0, 1):
                    continue
                direction = self.DIRECTIONS[(self.heading + relative) % 4]
                confirmations = getattr(relative_walls, "confirmed_edges", None)
                if (tile["known"].get(direction, False) and tile[direction] != value
                        and confirmations is not None and relative not in confirmations):
                    continue
                if not tile["known"].get(direction, False) or tile[direction] != value:
                    if tile["known"].get(direction, False):
                        print(f"[MAP REOBSERVE] ({self.x},{self.y}) {direction}: "
                              f"{tile[direction]} -> {value}（2回一致の観測を採用）")
                    self._wall(self.x, self.y, direction, int(value))
                    changed = True
            if changed:
                self.save()

    def mapping(self, x, y, A, B, C, D, E, W, S, N) -> None:
        values = (x, y, A, B, C, D, E, W, S, N)
        if any(not isinstance(v, int) for v in values):
            raise ValueError("mappingの各引数は整数にしてください")
        if A not in range(4) or B not in range(3) or C not in range(3):
            raise ValueError("A=0..3, B=0..2, C=0..2にしてください")
        if any(v not in (0, 1) for v in (D, E, W, S, N)):
            raise ValueError("D,E,W,S,Nは0か1にしてください")
        with self.lock:
            if self._suspended:
                return
            self._replace_tile(x, y, {"E": E, "W": W, "S": S, "N": N},
                               {"A": A, "B": B, "C": C, "D": D})
            self.save()

    def begin_forward_segment(self) -> None:
        """新しい前進区間の起点を確定する（速度・停止の制御は変更しない）。"""
        with self.lock:
            if self._suspended:
                return
            self._segment_origin = tuple(self.position_mm)
            self._segment_entered_tiles.clear()

    def _set_forward_pose(self, displacement_mm: float, motion_source: str) -> None:
        """区間起点からの絶対移動量で位置を再計算し、二重加算を防ぐ。"""
        origin = self._segment_origin
        if origin is None:
            origin = tuple(self.position_mm)
        dx, dy = self.STEPS[self.heading]
        target_x = origin[0] + dx * displacement_mm
        target_y = origin[1] + dy * displacement_mm
        desired_x = math.floor(target_x / RUN_DISTANCE_MM + 0.5)
        desired_y = math.floor(target_y / RUN_DISTANCE_MM + 0.5)
        direction = self.DIRECTIONS[self.heading]
        # 停止時に16cmの境界を越えていたら次タイルへ移る。
        # 5cmは「停止時の更新許可距離」であり、タイルサイズは32cmのまま。
        while (self.x, self.y) != (desired_x, desired_y):
            remaining = (desired_x - self.x) * dx + (desired_y - self.y) * dy
            old_x, old_y = self.x, self.y
            if remaining > 0:
                self.x += dx
                self.y += dy
                self._segment_entered_tiles.add((self.x, self.y))
                self._tile(self.x, self.y)
                self._wall(old_x, old_y, direction, 0)  # 実際に通過した辺は開放確定
            elif remaining < 0:
                # 停止後のLiDAR補正で境界の手前と分かった場合、
                # 走行中の暫定タイルから現在地を戻す。
                self._segment_entered_tiles.discard((old_x, old_y))
                self.x -= dx
                self.y -= dy
            else:
                break
        self.position_mm = [target_x, target_y]
        self.last_motion_source = motion_source
        self.partial_mm = (
            (target_x - self.x * RUN_DISTANCE_MM) * dx
            + (target_y - self.y * RUN_DISTANCE_MM) * dy
        )

    def _replace_current_observation(self, relative_walls: dict) -> None:
        absolute_walls: dict[str, Optional[int]] = {
            direction: None for direction in self.DIRECTIONS
        }
        for relative, value in relative_walls.items():
            if relative not in (0, 1, 2, 3):
                continue
            direction = self.DIRECTIONS[(self.heading + relative) % 4]
            confirmations = getattr(relative_walls, "confirmed_edges", None)
            existing = self.tiles.get((self.x, self.y))
            if (existing is not None and existing["known"].get(direction, False)
                    and existing[direction] != value and confirmations is not None
                    and relative not in confirmations):
                continue
            if value is not None:
                absolute_walls[direction] = int(value)
        if (self.x, self.y) in self._segment_entered_tiles:
            # 新しいタイルへ実際に通過した入口はまず開放として記録する。
            rear = self.DIRECTIONS[(self.heading + 2) % 4]
            absolute_walls[rear] = 0
        self.visited.add((self.x, self.y))
        self._replace_tile(self.x, self.y, absolute_walls)

    def finish_forward(self, distance_mm: float, relative_walls: dict,
                       lidar_distance_mm: Optional[float] = None,
                       camera_colors: tuple[str, ...] = ()) -> None:
        with self.lock:
            if self._suspended:
                return
            # 停止後だけ呼ばれる。初期位置の観測は0mmでも一度作成する。
            # 以降は区間で5cm未満の移動なら旧タイル・座標・JSONを一切変更しない。
            initial_zero_observation = not self.tiles and distance_mm == 0.0
            if distance_mm < MAP_MIN_ADVANCE_MM and not initial_zero_observation:
                self._segment_origin = None
                self._segment_entered_tiles.clear()
                return
            use_lidar = lidar_distance_mm is not None
            if (use_lidar and not ENABLE_EXPERIMENTAL_BUMP_RECOVERY
                    and (not math.isfinite(lidar_distance_mm)
                         or abs(lidar_distance_mm - distance_mm) > 80.0)):
                print(f"[MAP ODOMETRY] 大きすぎる補正を棄却: encoder={distance_mm:.1f}, "
                      f"lidar={lidar_distance_mm}。エンコーダーで座標更新")
                use_lidar = False
            displacement = lidar_distance_mm if use_lidar else distance_mm
            origin_tile = (self.x, self.y)
            self._set_forward_pose(displacement,
                                   "LIDAR" if use_lidar else "ENCODER")
            destination_tile = (self.x, self.y)
            if destination_tile != origin_tile:
                # 停止時にのみ、推定したタイル境界の通過を記録する。
                # 方位・座標に基づいて各辺を1つずつ追加し、
                # Uターン帰路などでセンサーの壁検出が欠けてもBFSを維持する。
                dx, dy = self.STEPS[self.heading]
                cursor = origin_tile
                while cursor != destination_tile:
                    neighbor = (cursor[0] + dx, cursor[1] + dy)
                    # 座標推定が食い違ったときは推測で通行可能を作らない。
                    if ((destination_tile[0] - neighbor[0]) * dx
                            + (destination_tile[1] - neighbor[1]) * dy) < 0:
                        break
                    passage = frozenset((cursor, neighbor))
                    if passage not in self.confirmed_passages:
                        print(f"[NAV MAP] 実走行の接続を記録: {cursor} <-> {neighbor}")
                    self.confirmed_passages.add(passage)
                    cursor = neighbor
            self._replace_current_observation(relative_walls)
            if camera_colors:
                dx, dy = self.STEPS[self.heading]
                camera_tile = (
                    math.floor((self.position_mm[0] + dx * COLOR_VIEW_FORWARD_MM) / RUN_DISTANCE_MM + 0.5),
                    math.floor((self.position_mm[1] + dy * COLOR_VIEW_FORWARD_MM) / RUN_DISTANCE_MM + 0.5),
                )
                tile = self.tiles.get(camera_tile)
                if tile is None:
                    # 色だけ見えた未登録座標を探索用タイルには追加しない。
                    self.pending_camera_colors.setdefault(camera_tile, list(dict.fromkeys(camera_colors)))
                elif "camera_colors" not in tile:
                    pending = self.pending_camera_colors.pop(camera_tile, None)
                    tile["camera_colors"] = pending if pending is not None else list(dict.fromkeys(camera_colors))
            self._segment_origin = None
            self._segment_entered_tiles.clear()
            self.save()
            print(f"MAP STOP: ({self.x},{self.y}) heading={self.DIRECTIONS[self.heading]} "
                  f"travelled={distance_mm / 10:.1f} cm "
                  f"partial={self.partial_mm:.1f} mm (fresh tile overwrite)")

    def turn(self, direction: str, angle_deg: int, measured_deg: Optional[float] = None) -> None:
        if direction not in ("LEFT", "RIGHT") or angle_deg not in (90, 180):
            raise ValueError("旋回はLEFT/RIGHT、90/180度を指定してください")
        with self.lock:
            if self._suspended:
                return
            target = angle_deg if direction == "LEFT" else -angle_deg
            # 地図の方位は旋回指令で更新。実機トリムやLiDAR誤差では変更しない。
            signed_quarters = angle_deg // 90 * (1 if direction == "LEFT" else -1)
            self.heading = (self.heading - signed_quarters) % 4
            self.heading_angle_deg = (self.heading_angle_deg - target) % 360.0
            self.heading_confirmation = {"target_deg": target, "lidar_deg": measured_deg,
                                         "recorded_turn_deg": signed_quarters * 90,
                                         "source": "COMMAND"}
            self.heading_status = "COMMAND_BASED"
            self.save()

    def begin_turn(self) -> None:
        with self.lock:
            if self._suspended:
                return
            self.heading_status = "UNCONFIRMED"
            self.save()


    def correct_forward_angle(self, previous_angle_deg: float,
                              measured_deg: Optional[float]) -> None:
        with self.lock:
            if self._suspended:
                return
            if measured_deg is None:
                print("MAP ANGLE: 前進後の角度未確定、直前の方位を維持")
                return
            self.heading_angle_deg = (previous_angle_deg - measured_deg) % 360.0
            self.heading_confirmation = {
                "target_deg": 0.0, "lidar_deg": measured_deg,
                "recorded_turn_deg": 0, "source": "LIDAR_FORWARD",
            }
            self.save()
            print(f"MAP ANGLE: forward delta={measured_deg:+.1f}, "
                  f"heading={self.heading_angle_deg:.1f} deg")

    def snapshot(self) -> dict:
        with self.lock:
            return json.loads(json.dumps({
                "program_version": PROGRAM_VERSION,
                "pose": {"x": self.x, "y": self.y,
                         "heading": self.DIRECTIONS[self.heading],
                         "heading_status": self.heading_status,
                         "angle_deg": self.heading * 90,
                         "logical_angle_deg": self.heading * 90,
                         "estimated_body_angle_deg": self.heading_angle_deg,
                         "partial_mm": self.partial_mm},
                "position_mm": self.position_mm,
                "last_motion_source": self.last_motion_source,
                "heading_confirmation": self.heading_confirmation,
                "tile_size_mm": RUN_DISTANCE_MM,
                "tiles": list(self.tiles.values()),
                "pending_camera_colors": [
                    {"x": x, "y": y, "camera_colors": colors}
                    for (x, y), colors in self.pending_camera_colors.items()
                ],
                "visited_count": len(self.visited),
                "confirmed_passages": [
                    [list(p) for p in sorted(edge)]
                    for edge in sorted(self.confirmed_passages, key=lambda e: sorted(e))
                ],
            }))

    def save(self) -> None:
        if self._suspended:
            return
        if self.path is not None:
            payload = self.snapshot()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix(self.path.suffix + ".tmp")
            temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            os.replace(temporary, self.path)


tile_map = TileMap()


def mapping(x, y, A, B, C, D, E, W, S, N) -> None:
    """指定タイルの属性と東西南北の壁を保存する公開関数。"""
    tile_map.mapping(x, y, A, B, C, D, E, W, S, N)


def raw_corridor_is_clear(scan: Scan, bearing_deg: float,
                          distance_mm: float = RUN_DISTANCE_MM / 2.0 + TILE_WALL_MARGIN_MM) -> bool:
    """生LiDARの複数実測点で、現在タイル境界を越える空間を確認。

    Hough直線がないことだけでは「通れる」と判定しない。十分に遠い実測点を
    複数確認し、同じ扇形に近距離の実測点が存在しないときだけTrue。
    走行許可の代わりではなくfrontier計画時の追加根拠に使う。
    """
    points = scan.points
    if len(points) < 3:
        return False
    angle_diff = (points[:, 0] - bearing_deg + 180.0) % 360.0 - 180.0
    valid = (np.isfinite(points[:, 1]) & (points[:, 1] > 0)
             & (np.abs(angle_diff) <= 12.0))
    distances = points[valid, 1]
    bearings = angle_diff[valid]
    if len(distances) < 3:
        return False
    # 片側の遠距離点だけで真正面が空いていると誤判定しない。
    if not (np.any(bearings <= -3.0) and np.any(bearings >= 3.0)
            and np.any(np.abs(bearings) <= 4.0)):
        return False
    if np.any(distances < distance_mm):
        return False
    return int(np.count_nonzero(distances >= distance_mm)) >= 3


def raw_corridor_has_obstacle(scan: Scan, bearing_deg: float,
                              distance_mm: float = RUN_DISTANCE_MM + ROBOT_RADIUS_MM + 55.0) -> bool:
    """既存の±12度扇形の近距離点が複数ある場合だけ追加の拒否根拠にする。

    中心付近の柱反射や単発ノイズで全方向を塞がないよう、扇形を広げず
    3点以上を必要とする。斜めの壁は別途処理後Houghの壁判定で拒否する。
    """
    points = scan.points
    if not len(points):
        return False
    delta = (points[:, 0] - bearing_deg + 180.0) % 360.0 - 180.0
    distance = points[:, 1]
    valid = np.isfinite(distance) & np.isfinite(delta) & (distance > 0)
    return int(np.count_nonzero(valid & (distance < distance_mm)
                                & (np.abs(delta) <= 12.0))) >= 3


def navigation_directions(scan: Scan, m: TileMap,
                          walls: Optional[dict] = None) -> tuple[set[int], set[int]]:
    """最新のHough壁・進路の複数近距離点を優先し、開放と障害方向を分離する。"""
    opened: set[int] = set()
    blocked: set[int] = set()
    if walls is None:
        walls = classify_scan_walls(scan)
    with m.lock:
        heading = m.heading
    for relative in (0, 3, 1, 2):  # 前・左・右・後ろ
        bearing = float(relative * 90)
        absolute = (heading + relative) % 4
        if walls.get(relative) == 1 or raw_corridor_has_obstacle(
                scan, bearing, ROBOT_RADIUS_MM + STOP_COMMAND_CLEARANCE_MM):
            blocked.add(absolute)
        elif walls.get(relative) == 0 or raw_corridor_is_clear(scan, bearing):
            opened.add(absolute)
    return opened, blocked


def live_open_directions(scan: Scan, m: TileMap) -> set[int]:
    """マップを変更せず、障害方向を除いた現在位置の開放方角を返す。"""
    return navigation_directions(scan, m)[0]


def tile_boundary_clearance_mm(m: TileMap, relative: int) -> float:
    """車体端から現タイル境界まで。方位と中心からのずれを反映する。"""
    with m.lock:
        dx, dy = m.STEPS[(m.heading + relative) % 4]
        offset = ((m.position_mm[0] - m.x * RUN_DISTANCE_MM) * dx
                  + (m.position_mm[1] - m.y * RUN_DISTANCE_MM) * dy)
    return max(0.0, RUN_DISTANCE_MM / 2.0 - offset - ROBOT_RADIUS_MM) + TILE_WALL_MARGIN_MM


def classify_scan_walls(scan: Scan, front_stop: bool = False) -> dict:
    """停止・走行中共通。1回の最新LiDAR観測で4辺を再分類する。

    観測されない辺はNone（未確認）として返し、以前の観測は持ち越さない。
    """
    result = process_pipeline(scan.points[:, :2].astype(np.float32, copy=False))
    front = front_wall_clearance_mm(result.lines)
    left, right, _, _ = side_wall_clearances_mm(result.lines)
    rear_lines = [(SOURCE_SIZE - 1 - x1, SOURCE_SIZE - 1 - y1,
                   SOURCE_SIZE - 1 - x2, SOURCE_SIZE - 1 - y2)
                  for x1, y1, x2, y2 in result.lines]
    rear = front_wall_clearance_mm(rear_lines)

    def classify(distance, bearing_deg: float):
        boundary = tile_boundary_clearance_mm(tile_map, int(bearing_deg / 90) % 4)
        if distance is not None:
            return int(distance <= boundary)
        # Hough直線が見つからない=通路開放ではない。
        # 対応方向に十分遠い実測点が複数あり、近距離の障害物点が
        # 見えない場合だけ、「開放」として経路候補へ追加する。
        if not len(scan.points):
            return None
        # 細い前方扇形の十分に遠い実測点で、Hough不足による未確認を減らす。
        if raw_corridor_is_clear(scan, bearing_deg, boundary + ROBOT_RADIUS_MM):
            return 0
        return None

    return {0: 1 if front_stop else classify(front, 0.0),
            1: classify(right, 90.0), 2: classify(rear, 180.0),
            3: classify(left, 270.0)}


def observe_tile_walls(lidar: MS200, view, stopping: threading.Event,
                       front_stop: bool = False) -> dict:
    """新規辺は実測で記録。既知辺の変更だけに独立2回の一致を要求する。"""
    wait_after_turn(lidar, view, stopping, POINT_LIFETIME_SECONDS + 0.05)
    if stopping.is_set():
        return {}
    first = lidar.snapshot()
    check_lidar(lidar, first)
    # 停止理由だけで壁ありを強制せず、実際の2回の観測で判断する。
    first_walls = classify_scan_walls(first)
    wait_after_turn(lidar, view, stopping, POINT_LIFETIME_SECONDS + 0.05)
    if stopping.is_set():
        return {}
    second = lidar.snapshot()
    check_lidar(lidar, second)
    if getattr(first, "timestamp", None) is None or second.timestamp <= first.timestamp:
        print("[MAP REOBSERVE] 2回目が未更新。新規辺のみ記録し、旧地図の訂正は保留")
        return ObservedWalls(first_walls)
    second_walls = classify_scan_walls(second)
    confirmed = {d for d in range(4) if first_walls.get(d) in (0, 1)
                 and first_walls.get(d) == second_walls.get(d)}
    values = {d: second_walls.get(d) if second_walls.get(d) in (0, 1)
              else first_walls.get(d) for d in range(4)}
    print(f"[MAP REOBSERVE] 実測={values}, 訂正を許可する2回一致辺={sorted(confirmed)}")
    return ObservedWalls(values, confirmed)


def longitudinal_walls_unlimited(scan: Scan) -> tuple[dict[str, float], Optional[float]]:
    """全有効距離の前後壁を側壁の方向に揃えて測る。描画範囲は使わない。"""
    polar = np.asarray(scan.points)
    valid = (np.isfinite(polar[:, 0]) & np.isfinite(polar[:, 1]) & (polar[:, 1] > 0))
    xy = polar_to_xy(polar[valid])
    angle, _ = measure_sidewall_parallel_unlimited(scan)
    if angle is not None and abs(angle) > 20.0:
        return {}, angle
    radians = math.radians(0.0 if angle is None else angle)
    across = xy[:, 0] * math.cos(radians) - xy[:, 1] * math.sin(radians)
    along = xy[:, 0] * math.sin(radians) + xy[:, 1] * math.cos(radians)
    result = {}
    for name, sign in (("front", 1), ("rear", -1)):
        mask = (np.abs(across) <= 250.0) & (along * sign > ROBOT_RADIUS_MM + 25.0)
        u, v = along[mask] * sign, across[mask]
        order = np.argsort(u)
        u, v = u[order], v[order]
        # 薄い同一平面に6点・横幅80mm以上。側壁や単発点は採用しない。
        for i in range(len(u)):
            end = int(np.searchsorted(u, u[i] + 40.0, side="right"))
            if end - i >= 6 and float(np.ptp(v[i:end])) >= 80.0:
                result[name] = float(np.median(u[i:end]))
                break
    return result, angle


def estimate_forward_motion_unlimited(reference: Scan, current: Scan,
                                      encoder_mm: float) -> Optional[tuple[float, float]]:
    """前壁の接近量と後壁の離隔量が一致した場合だけ実移動量として採用する。"""
    before, before_angle = longitudinal_walls_unlimited(reference)
    after, after_angle = longitudinal_walls_unlimited(current)
    if not all(side in before and side in after for side in ("front", "rear")):
        return None
    distances = [before["front"] - after["front"], after["rear"] - before["rear"]]
    if abs(distances[0] - distances[1]) > 35.0:
        return None
    distance = float(np.mean(distances))
    if distance < -20.0 or distance > encoder_mm + max(80.0, encoder_mm * 0.25):
        return None
    yaw = 0.0 if before_angle is None or after_angle is None else after_angle - before_angle
    if abs(yaw) > 14.0:
        return None
    distance = max(0.0, distance)
    print(f"[LIDAR TRAVEL] all ranges={distance:.1f}mm, encoder={encoder_mm:.1f}mm")
    return distance, yaw


def remaining_to_requested_center_mm(m: TileMap, target: tuple[int, int]) -> float:
    """既に目的タイルへ入っていても、中心までの前進不足量を求める。"""
    with m.lock:
        dx, dy = m.STEPS[m.heading]
        return ((target[0] * RUN_DISTANCE_MM - m.position_mm[0]) * dx
                + (target[1] * RUN_DISTANCE_MM - m.position_mm[1]) * dy)


def estimate_forward_motion(reference: Scan, current: Scan,
                             encoder_mm: float) -> Optional[tuple[float, float]]:
    """前進前後のHough直線を照合し、移動量と角度変化を求める。"""
    if ENABLE_EXPERIMENTAL_BUMP_RECOVERY:
        all_range = estimate_forward_motion_unlimited(reference, current, encoder_mm)
        if all_range is not None:
            return all_range
    reference_lines = capture_hough_scan(reference)
    current_lines = capture_hough_scan(current)
    pixels = sample_hough_lines(current_lines)
    mm_per_pixel = REAL_SIZE_MM / SOURCE_SIZE
    center = SOURCE_SIZE // 2
    current_xy = np.column_stack(((pixels[:, 0] - center) * mm_per_pixel,
                                  (center - pixels[:, 1]) * mm_per_pixel))
    if not len(reference_lines.lines) or len(current_xy) < 12:
        return None
    binary = hough_line_image(reference_lines)
    if np.count_nonzero(binary) < 40:
        return None
    distance_image = cv2.distanceTransform(255 - binary, cv2.DIST_L2, 3)
    mm_per_pixel = REAL_SIZE_MM / SOURCE_SIZE
    candidates = []
    # 通常は元の±8cm探索へ戻す。0mmへの誤照合で原点に留まらない。
    minimum_mm = 0.0 if ENABLE_EXPERIMENTAL_BUMP_RECOVERY else max(0.0, encoder_mm - 80.0)
    for forward_mm in np.arange(minimum_mm, encoder_mm + 81.0, 5.0):
        best_score = float("inf")
        best_angle = 0.0
        for angle in range(-14, 15, 2):
            radians = math.radians(angle)
            rotation = np.array([[math.cos(radians), -math.sin(radians)],
                                 [math.sin(radians), math.cos(radians)]])
            rotated = current_xy @ rotation.T
            for side_mm in (-20.0, 0.0, 20.0):
                transformed = rotated + [side_mm, forward_mm]
                px = np.rint(SOURCE_SIZE // 2 + transformed[:, 0] / mm_per_pixel).astype(int)
                py = np.rint(SOURCE_SIZE // 2 - transformed[:, 1] / mm_per_pixel).astype(int)
                valid = (px >= 0) & (px < SOURCE_SIZE) & (py >= 0) & (py < SOURCE_SIZE)
                if np.count_nonzero(valid) < max(40, len(current_xy) * 0.40):
                    continue
                errors = distance_image[py[valid], px[valid]] * mm_per_pixel
                score = float(np.mean(np.minimum(errors, 40.0)))
                if score < best_score:
                    best_score = score
                    best_angle = float(angle)
        if math.isfinite(best_score):
            candidates.append((best_score, float(forward_mm), best_angle))
    if not candidates:
        return None
    best_score, best_distance, best_angle = min(candidates)
    alternatives = [score for score, distance, angle in candidates
                    if abs(distance - best_distance) >= 40.0]
    # 4cm異なる移動量も同程度に合うなら、縦方向の移動を特定できない。
    if best_score > 15.0 or not alternatives or min(alternatives) - best_score < 2.0:
        print(f"MAP LiDAR: ambiguous, score={best_score:.1f}")
        return None
    # x右・y前の平面で、現スキャンを開始座標へ戻す回転は車体左旋回と同符号。
    measured_rotation = best_angle
    print(f"MAP LiDAR: displacement={best_distance:.1f} mm, "
          f"rotation={measured_rotation:+.1f} deg, score={best_score:.1f}")
    return best_distance, measured_rotation


# Color detection from robocup_usb_color.py.
# OpenCV hue is 0..179, saturation/value 0..255.
COLOR_HSV_BANDS = {
    "red": ((0, 12), (170, 179)),  # wraps across hue=0
    "yellow": ((18, 35),),
    "green": ((38, 85),),
}
COLOR_BGR = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0)}
COLOR_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def analyze_color_frame(
    frame: np.ndarray,
    *,
    min_area: int = 350,
    min_saturation: int = 85,
    min_value: int = 60,
    roi: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0),
) -> list[dict]:
    """Return detections with color, area(px^2), bbox(x,y,w,h), center(x,y).

    The largest connected region is kept per color. A color may be reported
    alongside others. ROI coordinates are fractional positions in the image.
    """
    if frame is None or frame.size == 0:
        return []
    height, width = frame.shape[:2]
    rx1, ry1, rx2, ry2 = roi
    x1, y1 = int(width * rx1), int(height * ry1)
    x2, y2 = int(width * rx2), int(height * ry2)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("Invalid ROI")
    subframe = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(subframe, cv2.COLOR_BGR2HSV)
    results: list[dict] = []
    for color, ranges in COLOR_HSV_BANDS.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hue_lo, hue_hi in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(
                hsv,
                (hue_lo, min_saturation, min_value),
                (hue_hi, 255, 255),
            ))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, COLOR_MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, COLOR_MORPH_KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < min_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(largest)
        results.append({
            "color": color,
            "area": int(round(area)),
            "bbox": [bx + x1, by + y1, bw, bh],
            "center": [bx + x1 + bw // 2, by + y1 + bh // 2],
        })
    return sorted(results, key=lambda d: d["area"], reverse=True)


def annotate_color_frame(frame: np.ndarray, results: list[dict], stable: set) -> np.ndarray:
    image = frame.copy()
    for d in results:
        x, y, w, h = d["bbox"]
        color = d["color"]
        cv2.rectangle(image, (x, y), (x + w, y + h), COLOR_BGR[color], 2)
        label = f"{color.upper()} {d['area']}px" + (" OK" if color in stable else "")
        cv2.putText(image, label, (x, max(20, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_BGR[color], 2)
    return image


def open_color_camera(source: str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    device = source if source.startswith("/dev/") else int(source)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"Camera could not be opened: {source}")
    # MJPG may reduce USB bandwidth. Unsupported settings are simply ignored.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class ColorCamera:
    """USBカメラを単独所有し、画像と確定色を走行処理へ非同期で渡す。"""

    def __init__(self, source: str, roi: tuple[float, float, float, float]):
        self.source = source
        self.roi = roi
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.history: deque[tuple[float, set[str]]] = deque(maxlen=7)
        self.message = "カメラ起動中"
        self.frame: Optional[bytes] = None
        self.revision = 0
        self.thread = threading.Thread(target=self._capture, name="usb-color-camera", daemon=True)

    def _stable_colors(self, since: float = 0.0) -> tuple[str, ...]:
        cutoff = max(since, time.monotonic() - 1.0)
        samples = [colors for stamp, colors in self.history if stamp >= cutoff]
        votes = {color: sum(color in sample for sample in samples) for color in COLOR_HSV_BANDS}
        confirmed = (color for color, count in votes.items() if count >= 5)
        return tuple(sorted(confirmed, key=votes.__getitem__, reverse=True))

    def colors_since(self, since: float) -> tuple[str, ...]:
        with self.lock:
            return self._stable_colors(since)

    def snapshot(self) -> dict:
        with self.lock:
            fresh = bool(self.history and time.monotonic() - self.history[-1][0] <= 1.0)
            return {"message": self.message if fresh or not self.history else "カメラ映像を待機中",
                    "colors": self._stable_colors(), "revision": self.revision,
                    "has_image": fresh and self.frame is not None}

    def _capture(self) -> None:
        """USBカメラを単独所有し、色判定10HzとWeb映像30Hzを分離。"""
        cap = None
        try:
            cap = open_color_camera(self.source, 640, 480, CAMERA_PREVIEW_FPS)
            next_color = 0.0
            next_preview = 0.0
            detections: list[dict] = []
            stable: set[str] = set()
            failures = 0
            while not self.stopping.is_set():
                # この実機では30/60fps要求を約120fpsへ丸める。
                # grabを継続して古いフレームが溜まらないようにする。
                if not cap.grab():
                    failures += 1
                    if failures >= 20:
                        raise RuntimeError("カメラ映像を取得できません")
                    self.stopping.wait(0.02)
                    continue
                stamp = time.monotonic()
                if stamp < next_color and stamp < next_preview:
                    continue
                ok, frame = cap.retrieve()
                if not ok or frame is None or not frame.size:
                    failures += 1
                    if failures >= 20:
                        raise RuntimeError("カメラ映像を取得できません")
                    continue
                failures = 0
                if stamp >= next_color:
                    detections = analyze_color_frame(frame, roi=self.roi)
                    with self.lock:
                        self.history.append((stamp, {item["color"] for item in detections}))
                        stable = set(self._stable_colors())
                    next_color = stamp + COLOR_CAPTURE_INTERVAL_SECONDS
                if stamp >= next_preview:
                    marked = annotate_color_frame(frame, detections, stable)
                    encoded, jpeg = cv2.imencode(
                        ".jpg", marked, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    if encoded:
                        with self.lock:
                            self.message = "ライブ映像・色判定中"
                            self.frame = jpeg.tobytes()
                            self.revision += 1
                    next_preview = stamp + 1.0 / CAMERA_PREVIEW_FPS
        except (ValueError, RuntimeError, OSError, cv2.error) as exc:
            with self.lock:
                self.history.clear()
                self.frame = None
                self.message = f"カメラエラー {exc}"
            print(f"[COLOR CAMERA] {self.source}: {exc}", flush=True)
        finally:
            if cap is not None:
                cap.release()

class SharedView:
    def __init__(self):
        self.lock = threading.Lock()
        placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Waiting for MS200 LiDAR...", (55, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        ok, encoded = cv2.imencode(".jpg", placeholder, [cv2.IMWRITE_JPEG_QUALITY, 75])
        self.frame: Optional[bytes] = encoded.tobytes() if ok else None
        self.turn_frames: dict[str, bytes] = {}
        self.turn_revision = 0
        self.turn_error_deg: Optional[float] = None
        self.turn_message = "旋回待機中"

    def update_turn_scan(self, name: str, scan) -> None:
        result = capture_hough_scan(scan)
        binary = hough_line_image(result)
        image = add_grid(np.zeros_like(binary))
        image[binary > 0] = (0, 255, 255)
        cv2.putText(image, f"{name.upper()} HOUGH lines={len(result.lines)}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.turn_frames[name] = encoded.tobytes()
                self.turn_revision += 1

    def reset_turn(self) -> None:
        with self.lock:
            self.turn_frames.clear()
            self.turn_error_deg = None
            self.turn_message = "想定点群を作成中"
            self.turn_revision += 1

    def turn_snapshot(self) -> dict:
        with self.lock:
            return {"revision": self.turn_revision, "error_deg": self.turn_error_deg,
                    "message": self.turn_message}

    def set_turn_message(self, message: str) -> None:
        with self.lock:
            self.turn_message = message

    def update(self, frame: bytes) -> None:
        with self.lock:
            self.frame = frame

    def get(self) -> Optional[bytes]:
        with self.lock:
            return self.frame


LCD_SIZE = 240
GOAL_IMAGE_PATH = PROJECT_ROOT / "Display_Process" / "Png" / "goal.png"
GOAL_DISPLAY_ACTIVE = threading.Event()


def render_goal_lcd() -> np.ndarray:
    """LCDマップの代わりにgoal.pngを240x240で表示。"""
    if GOAL_IMAGE_PATH.is_file():
        raw = np.fromfile(str(GOAL_IMAGE_PATH), dtype=np.uint8)
        image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
        if image is not None:
            return cv2.resize(image, (LCD_SIZE, LCD_SIZE), interpolation=cv2.INTER_AREA)
    # goal.png未設置でもモーターは再始動しない。
    fallback = np.zeros((LCD_SIZE, LCD_SIZE, 3), dtype=np.uint8)
    cv2.putText(fallback, "GOAL", (31, 122), cv2.FONT_HERSHEY_SIMPLEX,
                1.9, (10, 225, 255), 4, cv2.LINE_AA)
    return fallback


def publish_goal_lcd() -> None:
    GOAL_DISPLAY_ACTIVE.set()
    image = render_goal_lcd()
    ok, png = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("GOAL画像PNGエンコード失敗")
    save_display_mirror(png.tobytes())
    if not GOAL_IMAGE_PATH.is_file():
        print(f"[GOAL] {GOAL_IMAGE_PATH} が未配置。内蔵GOAL画面を使用")

LCD_TEXT = cv2.FONT_HERSHEY_SIMPLEX


def _lcd_decode(payload: Optional[bytes]) -> Optional[np.ndarray]:
    if not payload:
        return None
    return cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)


def _lcd_fit(canvas: np.ndarray, source: Optional[np.ndarray],
             x: int, y: int, width: int, height: int) -> None:
    """画像比率を維持して指定領域へ収める。"""
    if source is None:
        return
    src_h, src_w = source.shape[:2]
    if not src_w or not src_h:
        return
    scale = min(width / src_w, height / src_h)
    w, h = max(1, round(src_w * scale)), max(1, round(src_h * scale))
    interpolation = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
    resized = cv2.resize(source, (w, h), interpolation=interpolation)
    left = x + (width - w) // 2
    top = y + (height - h) // 2
    canvas[top:top + h, left:left + w] = resized


def render_lcd_tile_map(snapshot: dict, size: int = LCD_SIZE) -> np.ndarray:
    """Web /map.json の状態をLCD向けに同じ方位・色で描画する。"""
    img = np.full((size, size, 3), (43, 37, 32), dtype=np.uint8)
    pose = snapshot["pose"]
    tiles = snapshot["tiles"]
    pending = snapshot.get("pending_camera_colors", [])
    xs = [0, pose["x"]] + [t["x"] for t in tiles + pending]
    ys = [0, pose["y"]] + [t["y"] for t in tiles + pending]
    min_x, max_x = min(xs) - 1, max(xs) + 1
    min_y, max_y = min(ys) - 1, max(ys) + 1
    cell = min((size - 10) / (max_x - min_x + 1),
               (size - 36) / (max_y - min_y + 1))
    if cell < 5:
        cell = 5
        half = max(1, (size - 36) // round(cell) // 2)
        min_x, max_x = pose["x"] - half, pose["x"] + half
        min_y, max_y = pose["y"] - half, pose["y"] + half
    origin_x = (size - cell * (max_x - min_x + 1)) / 2
    origin_y = 20 + (size - 36 - cell * (max_y - min_y + 1)) / 2

    def pos(tx, ty):
        return (round(origin_x + (tx - min_x) * cell),
                round(origin_y + (max_y - ty) * cell))

    colors = [(67, 57, 49), (5, 5, 5), (204, 196, 191), (168, 91, 32)]
    for observation in pending:
        tx, ty = observation["x"], observation["y"]
        camera_color = COLOR_BGR.get(next(iter(observation.get("camera_colors", ())), None))
        if camera_color is not None and min_x <= tx <= max_x and min_y <= ty <= max_y:
            x, y = pos(tx, ty)
            cv2.rectangle(img, (x, y), (round(x + cell), round(y + cell)), camera_color, -1)
    for tile in tiles:
        tx, ty = tile["x"], tile["y"]
        if not (min_x <= tx <= max_x and min_y <= ty <= max_y):
            continue
        x, y = pos(tx, ty)
        right, bottom = round(x + cell), round(y + cell)
        camera_color = COLOR_BGR.get(next(iter(tile.get("camera_colors", ())), None))
        cv2.rectangle(img, (x, y), (right, bottom), camera_color or colors[tile["A"]], -1)
        border = (90, 90, 255) if tile["D"] else (124, 112, 102)
        cv2.rectangle(img, (x, y), (right, bottom), border, 1)
        edges = {"N": ((x, y), (right, y)),
                 "S": ((x, bottom), (right, bottom)),
                 "W": ((x, y), (x, bottom)),
                 "E": ((right, y), (right, bottom))}
        for direction, (a, b) in edges.items():
            if not tile["known"][direction]:
                # 未判定の壁は点線で表示
                length = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
                for step in range(0, length, 6):
                    r0 = step / max(length, 1)
                    r1 = min(step + 2, length) / max(length, 1)
                    cv2.line(img,
                             (round(a[0] + (b[0] - a[0]) * r0), round(a[1] + (b[1] - a[1]) * r0)),
                             (round(a[0] + (b[0] - a[0]) * r1), round(a[1] + (b[1] - a[1]) * r1)),
                             (142, 130, 120), 1)
            elif tile[direction]:
                cv2.line(img, a, b, (255, 255, 255), max(1, round(cell / 18)))
        if cell >= 28:
            ink = (17, 17, 17) if camera_color or tile["A"] == 2 else (238, 238, 238)
            cv2.putText(img, f'{tx},{ty}', (x + 2, y + 11), LCD_TEXT, 0.27, ink, 1)
    x, y = pos(pose["x"], pose["y"])
    center = (round(x + cell / 2), round(y + cell / 2))
    radius = max(3, round(cell * 0.27))
    cv2.circle(img, center, radius, (47, 37, 16), -1)
    cv2.circle(img, center, radius, (255, 226, 77), 1)
    angle = math.radians(pose["angle_deg"])
    tip = (round(center[0] + math.sin(angle) * radius),
           round(center[1] - math.cos(angle) * radius))
    cv2.arrowedLine(img, center, tip, (255, 226, 77), 2, tipLength=0.4)
    img[:18, :] = (18, 18, 18)
    img[size-16:, :] = (18, 18, 18)
    cv2.putText(img, f'MAP ({pose["x"]},{pose["y"]}) {pose["heading"]}',
                (5, 13), LCD_TEXT, 0.38, (255, 255, 255), 1)
    cv2.putText(img, 'N^ E>   dashed=unknown',
                (3, size-4), LCD_TEXT, 0.28, (210, 210, 210), 1)
    return img


def _lcd_view_snapshot(view: SharedView):
    with view.lock:
        return (view.frame, dict(view.turn_frames),
                view.turn_error_deg, view.turn_message, view.turn_revision)


def render_lcd_content(view: SharedView, mode: str, elapsed: float) -> np.ndarray:
    """LCDに表示するフレーム。Webで公開するのと同じ画像と地図データを使用。"""
    if GOAL_DISPLAY_ACTIVE.is_set():
        return render_goal_lcd()
    frame, turns, error, message, revision = _lcd_view_snapshot(view)
    if mode == "cycle":
        # LiDAR / Map / Turn の順で切り替える（周期は呼出元の引数から切替）
        mode = ("lidar", "map", "turn")[int(elapsed) % 3]
    if mode == "lidar":
        source = _lcd_decode(frame)
        result = np.full((LCD_SIZE, LCD_SIZE, 3), 14, dtype=np.uint8)
        _lcd_fit(result, source, 0, 0, LCD_SIZE, LCD_SIZE)
        return result
    if mode == "map":
        return render_lcd_tile_map(tile_map.snapshot())
    if mode == "turn":
        result = np.full((LCD_SIZE, LCD_SIZE, 3), 18, dtype=np.uint8)
        cv2.putText(result, 'TURN / Hough', (6, 17), LCD_TEXT, 0.47, (240, 240, 240), 1)
        for i, key in enumerate(("before", "expected", "after")):
            y = 22 + i * 71
            cv2.putText(result, key.upper(), (4, y + 11), LCD_TEXT,
                        0.32, (240, 240, 240), 1)
            _lcd_fit(result, _lcd_decode(turns.get(key, frame)), 81, y, 150, 68)
        if error is not None:
            cv2.putText(result, f'angle diff {error:+.1f} deg', (5, 237),
                        LCD_TEXT, 0.38, (0, 240, 240), 1)
        return result
    # web: 画面情報の配置を縮小し、LiDAR・Map・Turnを1画面に合成する。
    # ブラウザのHTML/CSSをスクリーンショットしているわけではない。
    result = np.full((LCD_SIZE, LCD_SIZE, 3), 17, dtype=np.uint8)
    cv2.putText(result, 'MS200 LiDAR + Tile Map', (5, 14),
                LCD_TEXT, 0.42, (255, 255, 255), 1)
    cv2.rectangle(result, (0, 19), (239, 151), (70, 70, 70), 1)
    _lcd_fit(result, _lcd_decode(frame), 2, 21, 145, 128)
    map_img = render_lcd_tile_map(tile_map.snapshot())
    _lcd_fit(result, map_img, 149, 31, 89, 91)
    cv2.putText(result, 'Map', (166, 29), LCD_TEXT, 0.30, (255, 255, 255), 1)
    for i, key in enumerate(("before", "expected", "after")):
        x = i * 80
        cv2.putText(result, key.upper(), (x + 3, 163), LCD_TEXT,
                    0.31, (235, 235, 235), 1)
        _lcd_fit(result, _lcd_decode(turns.get(key, frame)), x + 2, 166, 76, 69)
    return result


def lcd_mirror_loop(view: SharedView, stopping: threading.Event,
                    mode: str, fps: float, cycle_seconds: float) -> None:
    """Web表示に使う画像・地図からLCDプレビューPNGをRAMへ保存する。

    display_manager5.py がPNGを監視・表示するため、FRAME/IMAGEのソケットは不要。
    読めない場合でもLiDARやWebは継続する。
    """
    started = time.monotonic()
    last_payload: Optional[bytes] = None
    last_saved = 0.0
    unavailable = False
    while not stopping.is_set():
        began = time.monotonic()
        target_mode = mode
        if mode == "cycle":
            target_mode = ("lidar", "map", "turn")[
                int((began - started) / cycle_seconds) % 3
            ]
        try:
            # Webの /stream と /map.json で使っているデータを同一ソースから合成
            lcd = render_lcd_content(view, target_mode, began - started)
            ok, encoded = cv2.imencode(".png", lcd)
            if not ok:
                raise RuntimeError("LCDプレビューPNGのエンコードに失敗しました")
            payload = encoded.tobytes()
            # 静止画でも更新時刻を進め、LCD側の有効期限切れを防ぐ。
            if payload != last_payload or began - last_saved >= 1.0:
                save_display_mirror(payload)
                last_payload = payload
                last_saved = began
            if unavailable:
                print(f"[LCD] PNG保存を再開しました: {LCD_MIRROR_PATH}")
            unavailable = False
        except (OSError, RuntimeError, ValueError) as exc:
            if not unavailable:
                print(f"[LCD] PNG保存失敗（Webは継続）: {exc}")
            unavailable = True
            last_payload = None
            stopping.wait(3.0)
            continue
        stopping.wait(max(0.0, 1 / fps - (time.monotonic() - began)))


def start_web(view: SharedView, port: int, color_camera: Optional[ColorCamera] = None) -> uvicorn.Server:
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        return """<!doctype html><meta name=viewport content='width=device-width'>
<style>
html,body{margin:0;width:100%;min-height:100%;background:#111;color:#eee;text-align:center;font-family:sans-serif}
h3{height:5vh;line-height:5vh;margin:0;font-size:clamp(18px,2.2vw,30px)}
main{display:grid;grid-template-columns:2fr 1fr;gap:12px;padding:12px}
section{min-width:0;border:1px solid #555;border-radius:8px;padding:8px}
img{display:block;width:100%;height:auto;object-fit:contain;margin:0 auto}
[hidden]{display:none!important}
canvas{width:100%;height:65vh;background:#20252b}
pre{white-space:pre-wrap;text-align:left;font-size:12px}
.turn{grid-column:1/-1}.turn-images{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.turn-images figure{margin:0}.turn-images img{aspect-ratio:1;object-fit:contain}
@media(max-width:900px){main{grid-template-columns:1fr}}
</style>
<h3>MS200 LiDAR + Tile Map</h3><main>
<section><h3>LiDAR</h3><img src='/stream' onerror="this.src='/stream?t='+Date.now()"></section>
<section><h3>Map</h3><canvas id='map'></canvas><p id='pose'></p>
<p>北↑・東→ / 点線:未確認 / ○と矢印:現在地・進行方向</p><pre id='tile'></pre></section>
<section id='color-panel' hidden><h3>USBカメラ</h3><p id='color-state'></p>
<img id='color-image' src='/color/stream' alt='USBカメラのライブ色検出映像' hidden style='max-width:640px'></section>
<section class='turn'><h3>旋回のHough直線比較</h3><p id='turn-error'>旋回待機中</p>
<div class='turn-images'><figure><figcaption>旋回前</figcaption><img id='turn-before' src='/turn/before.jpg'></figure>
<figure><figcaption>旋回後の想定</figcaption><img id='turn-expected' src='/turn/expected.jpg'></figure>
<figure><figcaption>旋回後の実測</figcaption><img id='turn-after' src='/turn/after.jpg'></figure></div></section></main>
<script>
let turnRevision=-1;
const colorLabels={red:'赤',yellow:'黄',green:'緑'};
const cameraTileColors={red:'#f44',yellow:'#ff0',green:'#4f4'};
async function drawColors(){
 try{
 const state=await(await fetch('/color/state',{cache:'no-store'})).json();
 if(!state)return;
 document.getElementById('color-panel').hidden=false;
 document.getElementById('color-state').textContent=state.message+(state.colors.length?' / '+state.colors.map(c=>colorLabels[c]).join('・'):'');
 const image=document.getElementById('color-image');
 image.hidden=!state.has_image;
 }catch(e){}
 setTimeout(drawColors,250);
}drawColors();
async function drawMap(){
 try{
 const data=await(await fetch('/map.json',{cache:'no-store'})).json();
 const c=document.getElementById('map'),g=c.getContext('2d');
 c.width=Math.round(c.clientWidth*devicePixelRatio);c.height=Math.round(c.clientHeight*devicePixelRatio);
 const p=data.pose,tiles=data.tiles;
 const pending=data.pending_camera_colors||[],colorTiles=[...tiles,...pending];
 const xs=[0,p.x,...colorTiles.map(t=>t.x)],ys=[0,p.y,...colorTiles.map(t=>t.y)];
 const minX=Math.min(...xs)-1,maxX=Math.max(...xs)+1,minY=Math.min(...ys)-1,maxY=Math.max(...ys)+1;
 const s=Math.min(c.width/(maxX-minX+1),c.height/(maxY-minY+1));
 const ox=(c.width-s*(maxX-minX+1))/2,oy=(c.height-s*(maxY-minY+1))/2;
 const pos=(x,y)=>[ox+(x-minX)*s,oy+(maxY-y)*s];
 for(const t of pending){
  const color=cameraTileColors[t.camera_colors?.[0]];
  if(!color)continue;
  const [x,y]=pos(t.x,t.y);g.fillStyle=color;g.fillRect(x,y,s,s);
 }
 for(const t of tiles){
  const [x,y]=pos(t.x,t.y);
  const color=cameraTileColors[t.camera_colors?.[0]];
  g.fillStyle=color||['#313943','#050505','#bfc4cc','#205ba8'][t.A];g.fillRect(x,y,s,s);
  g.strokeStyle=t.D?'#ff5a5a':'#66707c';g.lineWidth=1;g.setLineDash([]);g.strokeRect(x,y,s,s);
  const edges={N:[x,y,x+s,y],S:[x,y+s,x+s,y+s],W:[x,y,x,y+s],E:[x+s,y,x+s,y+s]};
  for(const d of ['E','W','S','N']){
   if(t.known[d]&&t[d]===0)continue;
   g.setLineDash(t.known[d]?[]:[3,5]);g.strokeStyle=t.known[d]?'#fff':'#78828e';g.lineWidth=t.known[d]?4:1;
   const a=edges[d];g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(a[2],a[3]);g.stroke();
  }
  g.setLineDash([]);g.fillStyle=(color||t.A===2)?'#111':'#eee';g.font=`${Math.max(8,s*.14)}px sans-serif`;
  g.fillText(`${t.x},${t.y}`,x+s*.1,y+s*.3);g.fillText(`B${t.B} C${t.C}`,x+s*.1,y+s*.75);
 }
 const [x,y]=pos(p.x,p.y);g.setLineDash([]);g.strokeStyle='#4de2ff';g.lineWidth=3;
 const cx=x+s/2,cy=y+s/2,r=s*.27;
 g.beginPath();g.arc(cx,cy,r,0,Math.PI*2);g.fillStyle='#10252fee';g.fill();g.stroke();
 const radians=p.angle_deg*Math.PI/180;
 const v=[Math.sin(radians),-Math.cos(radians)];
 const tipX=cx+v[0]*r*.72,tipY=cy+v[1]*r*.72;
 g.beginPath();g.moveTo(cx-v[0]*r*.55,cy-v[1]*r*.55);g.lineTo(tipX,tipY);
 g.moveTo(tipX-v[0]*r*.4-v[1]*r*.3,tipY-v[1]*r*.4+v[0]*r*.3);
 g.lineTo(tipX,tipY);g.lineTo(tipX-v[0]*r*.4+v[1]*r*.3,tipY-v[1]*r*.4-v[0]*r*.3);g.stroke();
 document.getElementById('pose').textContent=`(${p.x},${p.y}) ${p.heading} / ${p.angle_deg.toFixed(1)}° / ${p.heading_status} / tile=${data.tile_size_mm/10}cm`;
 const t=tiles.find(t=>t.x===p.x&&t.y===p.y);document.getElementById('tile').textContent=t?JSON.stringify(t,null,2):'前進終了後に保存';
 const turn=await(await fetch('/turn/state',{cache:'no-store'})).json();
 if(turn.revision!==turnRevision){
  for(const key of ['before','expected','after'])document.getElementById('turn-'+key).src='/turn/'+key+'.jpg?r='+turn.revision;
  turnRevision=turn.revision;
 }
 document.getElementById('turn-error').textContent=turn.message+' / '+(turn.error_deg===null?'角度差: 未判定':`角度差: ${turn.error_deg.toFixed(1)}°`);
 }catch(e){document.getElementById('pose').textContent='Map: '+e.message;}
 setTimeout(drawMap,500);
}drawMap();
</script>"""

    @app.get("/map.json")
    async def map_snapshot():
        return Response(content=json.dumps(tile_map.snapshot()), media_type="application/json",
                        headers={"Cache-Control": "no-store"})

    @app.get("/color/state")
    async def color_state():
        return Response(content=json.dumps(color_camera.snapshot() if color_camera else None),
                        media_type="application/json", headers={"Cache-Control": "no-store"})

    @app.get("/color.jpg")
    async def color_image():
        if color_camera is not None:
            with color_camera.lock:
                frame = color_camera.frame
            if frame is not None:
                return Response(content=frame, media_type="image/jpeg", headers={"Cache-Control": "no-store"})
        return Response(status_code=404, headers={"Cache-Control": "no-store"})

    def camera_live_frames():
        # カメラ実体はColorCameraだけが開く。クライアントは最新JPEGを読む。
        last_revision = -1
        while color_camera is not None and not color_camera.stopping.is_set():
            with color_camera.lock:
                revision = color_camera.revision
                frame = color_camera.frame
            if frame is not None and revision != last_revision:
                last_revision = revision
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            else:
                time.sleep(0.015)

    @app.get("/color/stream")
    async def color_stream():
        if color_camera is None:
            return Response(status_code=503, content=b"USB camera disabled")
        return StreamingResponse(
            camera_live_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate",
                     "X-Accel-Buffering": "no"})

    @app.get("/turn/state")
    async def turn_state():
        return Response(content=json.dumps(view.turn_snapshot()), media_type="application/json",
                        headers={"Cache-Control": "no-store"})

    @app.get("/turn/{name}.jpg")
    async def turn_image(name: str):
        with view.lock:
            frame = view.turn_frames.get(name, view.frame)
        return Response(content=frame or b"", media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.get("/lcd.jpg")
    async def lcd_snapshot(mode: str = "web"):
        if mode not in ("web", "lidar", "map", "turn"):
            return Response(content=b"invalid mode", status_code=400)
        frame = render_lcd_content(view, mode, 0.0)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
        return Response(content=encoded.tobytes() if ok else b"",
                        media_type="image/jpeg", headers={"Cache-Control": "no-store"})

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

    @app.get("/lcd.png")
    async def lcd_preview():
        """LCDへ表示するのと全く同じ保存画像（ブラウザで動作確認用）。"""
        try:
            body = LCD_MIRROR_PATH.read_bytes()
        except OSError:
            return Response(status_code=503, content=b"LCD preview not ready")
        return Response(content=body, media_type="image/png",
                        headers={"Cache-Control": "no-store"})

    # ポートを事前に確保してからUvicornを起動する。起動時のbind競合を防ぎ、
    # 8000が他アプリで使われている場合は8001..8010へ順にフォールバックする。
    listener = None
    chosen_port = None
    for candidate in range(port, min(port + 10, 65535) + 1):
        trial = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            trial.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            trial.bind(("0.0.0.0", candidate))
            trial.listen(128)
        except OSError as exc:
            trial.close()
            if exc.errno == errno.EADDRINUSE:
                print(f"[WEB] port {candidate} は使用中。次のポートを試します。")
                continue
            raise RuntimeError(f"Webポート{candidate}を確保できません: {exc}") from exc
        listener = trial
        chosen_port = candidate
        break
    if listener is None:
        raise RuntimeError(
            f"Web用ポート{port}〜{min(port + 10, 65535)}がすべて使用中です。"
            " sudo ss -ltnp | grep ':800' で確認してください。"
        )

    assert chosen_port is not None
    server = uvicorn.Server(uvicorn.Config(app, host="0.0.0.0", port=chosen_port,
                                          log_level="warning"))
    server.actual_port = chosen_port
    thread = threading.Thread(target=server.run, kwargs={"sockets": [listener]},
                              name="web", daemon=True)
    try:
        thread.start()
        deadline = time.monotonic() + 5.0
        while not server.started:
            if not thread.is_alive():
                raise RuntimeError(f"Webサーバーを起動できません: port={chosen_port}")
            if time.monotonic() >= deadline:
                server.should_exit = True
                raise RuntimeError(f"Webサーバー起動タイムアウト: port={chosen_port}")
            time.sleep(0.05)
    except BaseException:
        listener.close()
        raise
    if chosen_port != port:
        print(f"[WEB] {port}が使用中のため{chosen_port}へ切り替えました。")
    print(f"[WEB] Web画面: http://<Raspberry PiのIP>:{chosen_port}/")
    server.robot_thread = thread  # 緊急再起動時に旧Webサーバーの終了を確認する
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




def estimate_lidar_rotation_deg(
    reference_lines: HoughScan,
    current_scan: Scan,
    expected_signed_deg: float,
) -> tuple[Optional[float], float, int]:
    """エンコーダー予測から回したHough直線と現在のHough直線を照合する。"""
    expected = rotate_hough_scan(reference_lines, expected_signed_deg)
    current = capture_hough_scan(current_scan)
    diagnostics = {}
    delta = small_rotation_angle(expected, current, LIDAR_TURN_SEARCH_WINDOW_DEG, diagnostics)
    score = float(diagnostics.get("score_mm", float("inf")))
    if delta is None:
        return None, score, len(current.lines)
    return expected_signed_deg + delta, score, len(current.lines)


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
) -> Optional[float]:
    """エンコーダーで範囲を絞り、LiDAR照合で目標角へ合わせて旋回する。"""
    target_counts = counts_per_90 * angle_deg / 90.0
    target_signed_deg = float(angle_deg if direction == "LEFT" else -angle_deg)
    print(
        f"旋回開始: direction={direction}, target={angle_deg} deg, "
        f"encoder_target={target_counts:.0f}, comparison=Hough lines"
    )
    reference_scan = lidar.snapshot()
    check_lidar(lidar, reference_scan)
    reference_lines = capture_hough_scan(reference_scan)
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
                reference_lines,
                scan,
                expected_signed_deg,
            )
            # 許容角は停止後の補正要否にだけ使う。通常旋回は目標への到達・通過で終了。
            reached_target = (lidar_rotation is not None and
                              (lidar_rotation if direction == "LEFT" else -lidar_rotation) >= angle_deg)
            if reached_target:
                finish_reason = "LIDAR TARGET REACHED"
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
    return lidar_rotation


def wait_after_turn(
    lidar: MS200,
    view: SharedView,
    stopping: threading.Event,
    seconds: float,
) -> None:
    """旋回前の古い点群が消えるまで停止し、新しい向きの画面を更新する。"""
    deadline = time.monotonic() + seconds
    next_render = 0.0
    while not stopping.is_set() and time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_render:
            render(lidar, view, 0.0, "TURN COMPLETE: LIDAR REFRESH")
            next_render = time.monotonic() + 0.20
        stopping.wait(max(0.0, min(0.02, deadline - time.monotonic())))








def small_rotation_angle(reference, current, search_degrees: int = 35,
                         diagnostics: Optional[dict] = None) -> Optional[float]:
    """Hough線分だけを回転・位置照合し、想定直線と実測直線の角度差を返す。"""
    original = capture_hough_scan(reference)
    observed = capture_hough_scan(current)
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update(reference_lines=len(original.lines), current_lines=len(observed.lines),
                           source="HOUGH_LINES")
    ref_points = sample_hough_lines(original)
    cur_points = sample_hough_lines(observed)
    if len(ref_points) < 12 or len(cur_points) < 12:
        if diagnostics is not None:
            diagnostics["reason"] = "比較できるHough直線が不足"
        return None
    ref_distance = cv2.distanceTransform(255 - hough_line_image(original), cv2.DIST_L2, 3)
    cur_distance = cv2.distanceTransform(255 - hough_line_image(observed), cv2.DIST_L2, 3)
    center = float(SOURCE_SIZE // 2)
    candidates = {}

    def evaluate(angle):
        radians = math.radians(angle)
        rotation = np.array([[math.cos(radians), -math.sin(radians)],
                             [math.sin(radians), math.cos(radians)]])
        rotated = (ref_points - center) @ rotation.T + center
        best = (float("inf"), 0.0, 0.0, 0.0)
        # バンプによる中心移動を角度差と混同しないよう、各軸±60mmを同時探索。
        for tx in (-36.0, -18.0, 0.0, 18.0, 36.0):
            for ty in (-36.0, -18.0, 0.0, 18.0, 36.0):
                shift = np.array([tx, ty])
                forward = np.rint(rotated + shift).astype(int)
                reverse = np.rint((cur_points - center - shift) @ rotation + center).astype(int)
                valid_f = np.all((forward >= 0) & (forward < SOURCE_SIZE), axis=1)
                valid_r = np.all((reverse >= 0) & (reverse < SOURCE_SIZE), axis=1)
                if valid_f.sum() < len(ref_points) * 0.4 or valid_r.sum() < len(cur_points) * 0.4:
                    continue
                f, r = forward[valid_f], reverse[valid_r]
                errors = np.concatenate((cur_distance[f[:, 1], f[:, 0]],
                                         ref_distance[r[:, 1], r[:, 0]]))
                coverage = float(np.mean(errors <= 6.0))
                clipped = np.sort(np.minimum(errors, 25.0))
                score = float(np.mean(clipped[:max(1, int(len(clipped) * 0.6))]))
                score += 4.0 * (1.0 - coverage)
                if score < best[0]:
                    best = (score, coverage, tx, ty)
        candidates[int(angle)] = best

    for angle in range(-search_degrees, search_degrees + 1, 3):
        evaluate(angle)
    # 粗探索の最良候補の周囲だけを1度刻みで調べる。
    coarse = min(candidates, key=lambda a: candidates[a][0])
    for angle in range(max(-search_degrees, coarse - 3), min(search_degrees, coarse + 3) + 1):
        evaluate(angle)
    angle = min(candidates, key=lambda a: candidates[a][0])
    score, coverage, tx, ty = candidates[angle]
    alternatives = [value[0] for a, value in candidates.items() if abs(a - angle) >= 12]
    margin = min(alternatives) - score if alternatives else 0.0
    if diagnostics is not None:
        diagnostics.update(best_angle=angle, score_mm=round(score * REAL_SIZE_MM / SOURCE_SIZE, 2),
                           coverage=round(coverage, 3),
                           translation_mm=[round(tx * REAL_SIZE_MM / SOURCE_SIZE, 1),
                                           round(ty * REAL_SIZE_MM / SOURCE_SIZE, 1)],
                           alternative_margin_mm=round(margin * REAL_SIZE_MM / SOURCE_SIZE, 2))
    if abs(angle) >= search_degrees - 1:
        if diagnostics is not None:
            diagnostics["reason"] = "最良角度が探索範囲の端にあり未確定"
        return None
    if not math.isfinite(score) or score > 8.0 or coverage < 0.45:
        if diagnostics is not None:
            diagnostics["reason"] = "Hough直線の一致不足"
        return None
    if margin < 0.6:
        # 再測定はせず、同程度に一致する有力候補だけの中央値を採用する。
        # 最良角付近だけ細かく探索しているため、他の候補付近も同じ1度刻みに揃える。
        promising = [a for a, value in candidates.items()
                     if math.isfinite(value[0]) and value[0] <= score + 0.6
                     and value[1] >= 0.45 and abs(a) < search_degrees - 1]
        for a in promising:
            for refined in range(max(-search_degrees + 2, a - 2),
                                 min(search_degrees - 2, a + 2) + 1):
                if refined not in candidates:
                    evaluate(refined)
        best_score = min(value[0] for a, value in candidates.items()
                         if math.isfinite(value[0]) and value[1] >= 0.45
                         and abs(a) < search_degrees - 1)
        eligible = sorted(a for a, value in candidates.items()
                          if math.isfinite(value[0]) and value[0] <= min(8.0, best_score + 0.6)
                          and value[1] >= 0.45 and abs(a) < search_degrees - 1)
        angle = float(np.median(eligible))
        if diagnostics is not None:
            diagnostics.update(selection="CANDIDATE_MEDIAN", candidate_angles_deg=eligible,
                               selected_angle_deg=angle)
        print(f"HOUGH: 角度候補が複数のため中央値を採用: {angle:+.1f} deg; candidates={eligible}")
    elif diagnostics is not None:
        diagnostics.update(selection="BEST_SCORE", selected_angle_deg=float(angle))
    return float(angle)


def front_turn_ready(lidar: MS200, stop_command_mm: float) -> bool:
    """処理後Houghの前壁が通常の旋回開始距離にあるか。生点群は使わない。"""
    scan = lidar.snapshot()
    check_lidar(lidar, scan)
    distance, _ = front_wall_measurement(capture_hough_scan(scan).lines, stop_command_mm)
    return distance is not None and distance <= stop_command_mm


def align_before_forward(motors: Motors, lidar: MS200, view: SharedView,
                         stopping: threading.Event, power: int,
                         stop_command_mm: float = STOP_COMMAND_CLEARANCE_MM) -> bool:
    """記録方位を目標とし、推定車体角との差をLiDAR閉ループで補正する。"""
    target = float(tile_map.heading * 90)
    estimated = tile_map.heading_angle_deg
    error = (target - estimated + 180.0) % 360.0 - 180.0
    # モーターの最小回転量より小さい誤差を追い込まない。
    if abs(error) <= INITIAL_HEADING_TOLERANCE_DEG:
        return True
    if front_turn_ready(lidar, stop_command_mm):
        motors.stop(force=True)
        print("ALIGN: 前壁が旋回距離内のため単独補正を省略。次の旋回へ角度差を反映")
        view.set_turn_message("前壁が近いため先行補正なし。次の旋回で角度を補正")
        return True
    try:
        for attempt in range(14):
            if stopping.is_set():
                return False
            error = (target - estimated + 180.0) % 360.0 - 180.0
            print(f"ALIGN TO HEADING: target={target:.0f}, estimated={estimated:.1f}, "
                  f"error={error:+.1f} deg, attempt={attempt + 1}")
            if abs(error) <= TURN_ALIGN_TOLERANCE_DEG:
                with tile_map.lock:
                    # 目標値を代入して合わせたことにせず、実測の残差をそのまま残す。
                    tile_map.heading_angle_deg = estimated
                    tile_map.heading_status = "ALIGNED"
                    tile_map.save()
                return True

            motors.stop(force=True)
            reference = lidar.snapshot()
            check_lidar(lidar, reference)
            # target/estimatedは時計回り正。実機の右旋回は左輪負・右輪正。
            signed_left_power = -power if error > 0 else power
            end = time.monotonic() + min(0.10, max(0.035, abs(error) * 0.004))
            motors.drive(signed_left_power, -signed_left_power, force=True)
            try:
                while not stopping.is_set() and time.monotonic() < end:
                    check_lidar(lidar, lidar.snapshot())
                    time.sleep(0.005)
            finally:
                motors.stop(force=True)

            measured_angles = []
            for _ in range(3):
                wait_after_turn(lidar, view, stopping, POINT_LIFETIME_SECONDS + 0.10)
                if stopping.is_set():
                    return False
                current = lidar.snapshot()
                check_lidar(lidar, current)
                delta = small_rotation_angle(reference, current)
                if delta is not None:
                    measured_angles.append(delta)
            if len(measured_angles) < 2 or max(measured_angles) - min(measured_angles) > 4.0:
                with tile_map.lock:
                    tile_map.heading_status = "UNCONFIRMED"
                    tile_map.save()
                print("ALIGN: 補正旋回の実角度を確認できないため前進せず停止")
                return False
            delta = float(np.median(measured_angles))
            estimated = (estimated - delta) % 360.0
            with tile_map.lock:
                tile_map.heading_angle_deg = estimated
                tile_map.heading_confirmation = {
                    "source": "LIDAR_ALIGNMENT", "target_heading_deg": target,
                    "measured_delta_deg": delta, "estimated_body_angle_deg": estimated,
                }
                tile_map.save()
            # 次のパルス前に、目標方位との残差を再計算する。
        print("ALIGN: 記録方位への角度補正が収束しないため前進せず停止")
        return False
    finally:
        motors.stop(force=True)


def expected_turn_scan(reference, rotation_deg: float) -> HoughScan:
    """抽出済みHough直線を回転。想定画像へ再度の前処理は適用しない。"""
    return rotate_hough_scan(capture_hough_scan(reference), rotation_deg)


def correct_once_by_encoder(motors: Motors, lidar: MS200,
                            stopping: threading.Event, residual: float,
                            power: int, counts_per_90: float,
                            timeout_seconds: float) -> Optional[float]:
    """1回のHough測定角を、再照合せずエンコーダーで回し切る。"""
    target_counts = abs(residual) / 90.0 * counts_per_90
    previous_left, previous_right = motors.encoder_pair()
    accumulated = 0.0
    # 残差が負なら左旋回不足。左輪正・右輪負で不足を補う。
    left_power = power if residual < 0 else -power
    deadline = time.monotonic() + timeout_seconds * max(1.0, abs(residual) / 90.0)
    motors.drive(left_power, -left_power, force=True)
    try:
        while not stopping.is_set():
            check_lidar(lidar, lidar.snapshot())
            if time.monotonic() >= deadline:
                print("TURN SINGLE: エンコーダー補正がタイムアウトしたため停止")
                return None
            left, right = motors.encoder_pair()
            accumulated += (
                abs(signed_encoder_delta(left, previous_left))
                + abs(signed_encoder_delta(right, previous_right))
            ) / 2.0
            previous_left, previous_right = left, right
            if accumulated >= target_counts:
                magnitude = accumulated / counts_per_90 * 90.0
                return magnitude if residual < 0 else -magnitude
            stopping.wait(CONTROL_INTERVAL_SECONDS)
        return None
    finally:
        motors.stop(force=True)


def align_to_expected_scan(motors: Motors, lidar: MS200, view: SharedView,
                           stopping: threading.Event, expected: HoughScan,
                           expected_rotation: float, power: int,
                           counts_per_90: float = TURN_90_ENCODER_COUNTS * TURN_90_SCALE,
                           timeout_seconds: float = TURN_TIMEOUT_SECONDS,
                           stop_command_mm: float = STOP_COMMAND_CLEARANCE_MM) -> bool:
    """旋回後に新鮮なHough点群を複数回照合し、必要なら少しずつ再補正。

    角度の2回測定が食い違う場合には誤旋回を防ぐため補正しない。
    Hough不足そのものでは探索を中止せず、エンコーダ旋回結果を採用する。
    """
    motors.stop(force=True)
    if stopping.is_set():
        return False
    target_heading = float(tile_map.heading * 90)
    for attempt in range(TURN_ALIGN_MAX_PASSES):
        if stopping.is_set():
            return False
        samples = []
        diagnostics = []
        near_wall = False
        for _ in range(2):
            stopping.wait(POINT_LIFETIME_SECONDS + 0.05)
            if stopping.is_set():
                return False
            scan = lidar.snapshot()
            check_lidar(lidar, scan)
            measured = capture_hough_scan(scan)
            view.update_turn_scan("after", measured)
            front, _ = front_wall_measurement(measured.lines, stop_command_mm)
            if front is not None and front <= stop_command_mm:
                near_wall = True
                break
            detail = {}
            error = small_rotation_angle(expected, measured, search_degrees=24,
                                         diagnostics=detail)
            diagnostics.append(detail)
            # 候補があっても一致率・平均残差が悪い画像では角度を採用しない。
            confidence_ok = (detail.get("coverage", 1.0) >= 0.55 and
                             detail.get("score_mm", 0.0) <= 12.0)
            if error is not None and confidence_ok:
                samples.append(float(error))
        if near_wall:
            view.set_turn_message("旋回後: 正面障害物が近いため追加旋回を省略")
            return True
        if len(samples) != 2:
            with tile_map.lock:
                tile_map.heading_status = "ENCODER_ONLY_HOUGH_UNAVAILABLE"
                tile_map.heading_confirmation = {
                    "source": "ENCODER_FALLBACK", "measurements_deg": samples,
                    "reason": "Hough点群不足/マッチング不成立",
                }
                tile_map.save()
            print(f"[TURN ALIGN] Hough不足、旋回補正を省略: {diagnostics}")
            view.set_turn_message("Hough不足: エンコーダ旋回で続行")
            return not stopping.is_set()
        if abs(samples[0] - samples[1]) > TURN_ALIGN_MEASUREMENT_SPREAD_DEG:
            print(f"[TURN ALIGN] 測定値不一致、追加旋回を省略: {samples}")
            view.set_turn_message("旋回後: LiDAR角度が不安定、補正省略")
            return not stopping.is_set()
        residual = float(np.median(samples))
        print(f"[TURN ALIGN] pass={attempt+1}, residual={residual:+.2f}° samples={samples}")
        with view.lock:
            view.turn_error_deg = residual
        if abs(residual) <= TURN_ALIGN_TOLERANCE_DEG:
            with tile_map.lock:
                tile_map.heading_status = "HOUGH_VERIFIED"
                tile_map.heading_angle_deg = target_heading
                tile_map.heading_confirmation = {
                    "source": "HOUGH_MULTI_VERIFIED", "residual_deg": residual,
                    "samples_deg": samples, "passes": attempt+1,
                }
                tile_map.save()
            view.set_turn_message(f"旋回角度確認: 誤差{residual:+.1f}°")
            return True
        if abs(residual) > TURN_ALIGN_MAX_TRUSTED_ERROR_DEG:
            print(f"[TURN ALIGN] 疑わしい大角度差 {residual:+.1f}° / 補正は行わない")
            view.set_turn_message("角度差が大きすぎて未確定、エンコーダ値を採用")
            return not stopping.is_set()
        step = float(np.clip(residual, -TURN_ALIGN_MAX_STEP_DEG,
                             TURN_ALIGN_MAX_STEP_DEG))
        applied = correct_once_by_encoder(
            motors, lidar, stopping, step, power, counts_per_90, timeout_seconds)
        if applied is None:
            view.set_turn_message("角度補正のモーター異常により停止")
            return False
        print(f"[TURN ALIGN] 補正 {applied:+.2f}°, 再測定")
        view.set_turn_message(f"旋回再補正: {applied:+.1f}° → 再測定")
    # 再測定が収束しなければ次へ盲目的な大旋回をしない。
    view.set_turn_message("旋回後: 補正回数上限。以降はエンコーダ推定で継続")
    with tile_map.lock:
        tile_map.heading_status = "HOUGH_CORRECTION_UNCONFIRMED"
        tile_map.save()
    return not stopping.is_set()


def measure_sidewall_parallel_unlimited(scan: Scan) -> tuple[Optional[float], dict]:
    """全距離の生LiDAR点群から側壁の角度を求める（Hough直線数に依存しない）。

    角度は、壁の x = m*y+b の atan(m)。正なら車体は壁に対して左へ傾き、
    右旋回で平行になる。LiDARの物理的な有効距離以上の推定はしない。
    """
    diagnostics: dict = {"source": "RAW_LIDAR_ALL_VALID_RANGES", "sides": {}}
    if len(scan.points) < TILE_PARALLEL_MIN_INLIERS:
        diagnostics["reason"] = "側壁推定に十分なLiDAR点群がない"
        return None, diagnostics
    polar = np.asarray(scan.points, dtype=np.float32)
    mask = np.isfinite(polar[:, 0]) & np.isfinite(polar[:, 1]) & (polar[:, 1] > 0)
    valid = polar[mask]
    if len(valid) < TILE_PARALLEL_MIN_INLIERS:
        diagnostics["reason"] = "有効距離の点群が少ない"
        return None, diagnostics
    xy = polar_to_xy(valid)
    # 距離上限(MAX_DISTANCE_MM/REAL_SIZE_MM)は意図的に適用しない。
    estimates: list[tuple[float, int, float, str]] = []
    rng = np.random.default_rng(20261024)
    for side_name, sign in (("left", -1), ("right", 1)):
        side = xy[xy[:, 0] * sign > ROBOT_RADIUS_MM + 25.0]
        if len(side) > 220:
            side = side[np.linspace(0, len(side)-1, 220, dtype=int)]
        if len(side) < TILE_PARALLEL_MIN_INLIERS:
            diagnostics["sides"][side_name] = "no_points"
            continue
        best = None
        # 水平な前壁や離れた点の偶然の組合せを側壁と誤認しないよう、
        # 壁方向±30度、200mm以上の縦方向スパン、7点以上の支持を要求する。
        for _ in range(min(190, max(90, 3 * len(side)))):
            i, j = rng.integers(0, len(side), 2)
            if i == j:
                continue
            x1, y1 = map(float, side[i]); x2, y2 = map(float, side[j])
            dy = y2 - y1
            if abs(dy) < TILE_PARALLEL_MIN_SPAN_MM:
                continue
            slope = (x2 - x1) / dy
            if abs(slope) > math.tan(math.radians(30.0)):
                continue
            intercept = x1 - slope * y1
            if intercept * sign <= ROBOT_RADIUS_MM + 20.0:
                continue
            residual = np.abs(side[:,0] - (slope * side[:,1] + intercept)) / math.hypot(1.0, slope)
            inside = residual <= TILE_PARALLEL_INLIER_MM
            n = int(inside.sum())
            if n < TILE_PARALLEL_MIN_INLIERS:
                continue
            inliers = side[inside]
            span = float(np.ptp(inliers[:,1]))
            if span < TILE_PARALLEL_MIN_SPAN_MM:
                continue
            # 必ず元の生点群に回帰。近い側壁をやや優先するが、最低7点が必要。
            fitted = np.polyfit(inliers[:,1], inliers[:,0], 1)
            slope_refined, intercept_refined = map(float, fitted)
            if abs(slope_refined) > math.tan(math.radians(30.0)):
                continue
            error = float(np.median(np.abs(inliers[:,0] - (slope_refined * inliers[:,1] + intercept_refined))))
            # 強い長い壁を優先し、類似の壁なら近いものを優先。
            merit = n * 2.0 + min(span, 1500.0) / 200.0 - error / 5.0 - abs(intercept_refined) / 500.0
            clearance = abs(intercept_refined) / math.hypot(1.0, slope_refined) - ROBOT_RADIUS_MM
            candidate = (merit, math.degrees(math.atan(slope_refined)), n, span, clearance)
            if best is None or merit > best[0]:
                best = candidate
        if best is None:
            diagnostics["sides"][side_name] = "insufficient_consistent_wall"
            continue
        _, angle, inliers, span, clearance = best
        diagnostics["sides"][side_name] = {
            "angle_deg": round(angle, 2), "inliers": inliers,
            "span_mm": round(span, 1), "clearance_mm": round(clearance, 1),
        }
        estimates.append((angle, inliers, span, side_name))
    if not estimates:
        diagnostics["reason"] = "距離制限なしでも信頼できる側壁が見つからない"
        return None, diagnostics
    if len(estimates) == 2 and abs(estimates[0][0] - estimates[1][0]) > 5.0:
        diagnostics["reason"] = "左右の側壁角度が一致せず未確定"
        return None, diagnostics
    angles = [v[0] for v in estimates]
    weights = [v[1] for v in estimates]
    angle = float(np.average(angles, weights=weights))
    diagnostics["angle_deg"] = round(angle,2)
    return angle, diagnostics


def estimate_tile_center_unlimited(scan: Scan) -> tuple[dict[str, Optional[float]], dict]:
    """生LiDAR全有効距離から4方向のタイル壁を確認して車体の中心ずれを求める。

    片側壁・未検出・想定タイル幅と合わない測定は採用しない。
    x=右寄り正、y=前寄り正（ロボット座標）。
    """
    xy = polar_to_xy(scan.points[np.isfinite(scan.points[:, 1]) &
                                  np.isfinite(scan.points[:, 0]) &
                                  (scan.points[:, 1] > 0)])
    distances: dict[str, float] = {}
    diagnostics: dict = {"source": "RAW_LIDAR_ALL_VALID_RANGES", "walls": {}}
    for name, axis, sign in (("right", 0, 1), ("left", 0, -1),
                             ("front", 1, 1), ("back", 1, -1)):
        if not len(xy):
            continue
        along = xy[:, axis] * sign
        cross = xy[:, 1 - axis]
        candidate = (np.abs(cross) <= 120) & (along >= 65) & (along <= 285)
        v = along[candidate]
        t = cross[candidate]
        if len(v) < 6:
            continue
        # 遠方や近隣別タイルの壁ではなく、最も近い安定平面を測定する。
        nearest = float(np.quantile(v, 0.17))
        inlier = np.abs(v - nearest) <= 25
        if int(inlier.sum()) < 6 or float(np.ptp(t[inlier])) < 55:
            continue
        d = float(np.median(v[inlier]))
        distances[name] = d
        diagnostics["walls"][name] = {"distance_mm": round(d, 1),
                                      "points": int(inlier.sum())}
    offsets: dict[str, Optional[float]] = {"x_mm": None, "y_mm": None}
    for axis, pos, neg, key in ((0, "right", "left", "x_mm"),
                                 (1, "front", "back", "y_mm")):
        if pos not in distances or neg not in distances:
            continue
        full_width = distances[pos] + distances[neg]
        if abs(full_width - RUN_DISTANCE_MM) > CENTERING_WALL_PAIR_TOLERANCE_MM:
            diagnostics[key] = f"壁間隔が不一致: {full_width:.1f}mm"
            continue
        value = (distances[neg] - distances[pos]) / 2.0
        if abs(value) <= CENTERING_MAX_ADJUST_MM:
            offsets[key] = value
        else:
            diagnostics[key] = f"中心ずれが想定より大きい: {value:.1f}mm"
    diagnostics["offsets_mm"] = offsets.copy()
    return offsets, diagnostics


def adopt_lidar_center_estimate(m: TileMap, offsets: dict[str, Optional[float]]) -> bool:
    """検証済みの現在タイル中心誤差のみ位置モデルへ反映。壁情報は更新しない。

    横方向の補正は次の移動方向の操舵補助、前方向の補正は
    distance_to_next_centerで次タイル中心までの移動距離に反映される。
    """
    with m.lock:
        if m._suspended or (m.x, m.y) not in m.visited:
            return False
        x, y = offsets.get("x_mm"), offsets.get("y_mm")
        if x is None and y is None:
            return False
        local_right = m.STEPS[(m.heading+1)%4]
        local_front = m.STEPS[m.heading]
        cx, cy = m.x * RUN_DISTANCE_MM, m.y * RUN_DISTANCE_MM
        # 計測できなかった軸は元のエンコーダ推定を維持する。
        if x is not None:
            old_right = ((m.position_mm[0] - cx) * local_right[0]
                         + (m.position_mm[1] - cy) * local_right[1])
        else:
            old_right = 0.0
        if y is not None:
            old_front = ((m.position_mm[0] - cx) * local_front[0]
                         + (m.position_mm[1] - cy) * local_front[1])
        else:
            old_front = 0.0
        m.position_mm[0] += local_right[0] * ((x - old_right) if x is not None else 0.0)
        m.position_mm[1] += local_right[1] * ((x - old_right) if x is not None else 0.0)
        m.position_mm[0] += local_front[0] * ((y - old_front) if y is not None else 0.0)
        m.position_mm[1] += local_front[1] * ((y - old_front) if y is not None else 0.0)
        dx, dy = m.STEPS[m.heading]
        m.partial_mm = ((m.position_mm[0] - cx) * dx
                        + (m.position_mm[1] - cy) * dy)
        m.last_motion_source = "LIDAR_CENTER_ESTIMATE"
        m.save()  # 停止状態だけで呼ぶ
        return True


def next_heading_lateral_offset_mm(m: TileMap, heading: int) -> float:
    """次の進行方向から見た車体の右寄り量を計算。"""
    with m.lock:
        right = m.STEPS[(heading + 1) % 4]
        cx, cy = m.x * RUN_DISTANCE_MM, m.y * RUN_DISTANCE_MM
        return (m.position_mm[0] - cx)*right[0] + (m.position_mm[1] - cy)*right[1]


def align_sidewalls_unlimited(motors: Motors, lidar: MS200, view: SharedView,
                              stopping: threading.Event, args: argparse.Namespace,
                              phase: str = "TILE") -> bool:
    """前進・通常旋回の終了後、生LiDAR全有効距離から側壁平行を補正する。

    壁が見えない/Hough直線が少ない場合は補正をスキップして進行する。
    LiDAR通信異常・モーター回転不能の場合は既存の安全停止を維持する。
    """
    label = f"{phase} ALIGN"
    tolerance_deg = (TURN_PARALLEL_TOLERANCE_DEG if phase == "TURN"
                     else TILE_PARALLEL_TOLERANCE_DEG)
    max_measured_deg = (TURN_PARALLEL_MAX_MEASURED_DEG if phase == "TURN"
                        else TILE_PARALLEL_MAX_MEASURED_DEG)
    motors.stop(force=True)
    for attempt in range(TILE_PARALLEL_RETRIES):
        if stopping.is_set():
            return False
        # 停止後の車体振動を待ち、旋回後は古いLiDAR点群が消えるのを待つ。
        stopping.wait(POINT_LIFETIME_SECONDS + 0.08)
        if stopping.is_set():
            return False
        angles = []
        last_diagnostic = {}
        for _ in range(2):
            if stopping.is_set():
                return False
            scan = lidar.snapshot()
            check_lidar(lidar, scan)
            if phase == "TURN":
                view.update_turn_scan("after", scan)  # 表示用Hough。補正角は全距離の生点から求める。
            # 回転時の接触を避けるため、極近接の正面障害物がある場合は補正しない。
            bearing = (scan.points[:,0] + 180.0) % 360.0 - 180.0
            close_front = (np.abs(bearing) <= 18.0) & (scan.points[:,1] > 0) & (
                scan.points[:,1] <= ROBOT_RADIUS_MM + args.stop_command_cm * 10.0)
            if np.count_nonzero(close_front) >= 3:
                print(f"[{label}] 正面に近接障害物。安全のため平行補正を省略")
                view.set_turn_message(f"{label}: 正面近接のため追加補正なし")
                return True
            angle, last_diagnostic = measure_sidewall_parallel_unlimited(scan)
            if angle is not None:
                angles.append(angle)
            stopping.wait(0.10)
        if len(angles) < 2 or abs(angles[0]-angles[1]) > 4.0:
            print(f"[{label}] 有効な角度測定が不十分。補正せず続行: {last_diagnostic}")
            view.set_turn_message(f"{label}: 側壁未確定、補正省略")
            return True
        measured = float(np.median(angles))
        print(f"[{label}] 全距離LiDAR側壁偏差={measured:+.2f} deg, attempt={attempt+1}")
        if abs(measured) <= tolerance_deg:
            with tile_map.lock:
                tile_map.heading_status = f"{phase}_PARALLEL"
                tile_map.heading_angle_deg = (tile_map.heading * 90 - measured) % 360.0
                tile_map.heading_confirmation = {
                    "source": "RAW_LIDAR_PARALLEL", "phase": phase,
                    "residual_deg": measured, "tolerance_deg": tolerance_deg,
                    "diagnostics": last_diagnostic,
                }
                tile_map.save()
            view.set_turn_message(f"{label}: 許容範囲内 {measured:+.1f}°、追加旋回なし")
            return True
        if abs(measured) > max_measured_deg:
            print(f"[{label}] 側壁偏差が大きすぎるため誤検出の可能性。省略: {measured:+.1f}°")
            return True
        correction = float(np.clip(measured, -TILE_PARALLEL_MAX_CORRECTION_DEG,
                                   TILE_PARALLEL_MAX_CORRECTION_DEG))
        # correct_once_by_encoder: 正の残差->右旋回、負->左旋回。
        applied = correct_once_by_encoder(
            motors, lidar, stopping, correction,
            min(ALIGNMENT_POWER, args.turn_power),
            args.turn_90_counts * args.turn_90_scale, args.turn_timeout_seconds,
        )
        if applied is None:
            print(f"[{label}] エンコーダ回転失敗。走行を中止")
            return False
        with tile_map.lock:
            tile_map.heading_angle_deg = (tile_map.heading * 90 - (measured + applied)) % 360.0
            tile_map.heading_status = f"{phase}_PARALLEL_CORRECTED"
            tile_map.heading_confirmation = {
                "source": "RAW_LIDAR_UNLIMITED_SIDEWALL", "phase": phase, "before_deg": measured,
                "applied_signed_left_deg":applied, "estimated_residual_deg":measured + applied,
            }
            tile_map.save()
        view.set_turn_message(f"{label}: {measured:+.1f}° -> 回転 {applied:+.1f}°")
    return not stopping.is_set()


def align_tile_end_unlimited(motors: Motors, lidar: MS200, view: SharedView,
                             stopping: threading.Event, args: argparse.Namespace) -> bool:
    """既存の前進停止後の側壁平行補正。通常旋回後と同じ処理を使う。"""
    return align_sidewalls_unlimited(motors, lidar, view, stopping, args, phase="TILE")


@dataclass(frozen=True)
class ExplorationPlan:
    """現在地から最短の未到達タイルまでの座標列。最初の要素は現在地。"""
    target: tuple[int, int]
    path: tuple[tuple[int, int], ...]
    known_walls: int
    bootstrap: bool


def _edge_key(a: tuple[int, int], b: tuple[int, int]) \
        -> frozenset[tuple[int, int]]:
    return frozenset((a, b))


def _map_edges(m: TileMap, node: tuple[int, int],
               live_open: Optional[set[int]] = None,
               blocked_edges: Optional[set[frozenset]] = None):
    """確定した開放辺＋現在タイルのみ生LiDARで裏付けた仮開放辺。"""
    x, y = node
    tile = m.tiles.get(node)
    if tile is None:
        return
    is_current = node == (m.x, m.y)
    for i, d in enumerate(m.DIRECTIONS):
        # 開放の仮候補は、現在地側の確定した壁を上書きしてはならない。
        if tile["known"].get(d, False) and tile[d] != 0:
            continue
        # スキャンの裏付けを得た現在地のみ「Hough未検出→不明」の辺を補う。
        from_live = is_current and live_open is not None and i in live_open
        if not from_live and (not tile["known"][d] or tile[d] != 0):
            continue
        dx, dy = m.DELTAS[d]
        other = (x + dx, y + dy)
        if blocked_edges and _edge_key(node, other) in blocked_edges:
            continue
        neighbor = m.tiles.get(other)
        opposite = m.OPPOSITE[d]
        # 隣接側で壁ありなら、生LiDARに通路らしく見えても通らない。
        if neighbor is not None and neighbor["known"][opposite] and neighbor[opposite] != 0:
            continue
        yield other


def _frontier_known_walls(m: TileMap, node: tuple[int, int]) -> int:
    """未到達マスの4辺のうち、到達済みタイルから判定できる辺の数。"""
    x, y = node
    count = 0
    for d in m.DIRECTIONS:
        dx, dy = m.DELTAS[d]
        neighbor = (x + dx, y + dy)
        if neighbor not in m.visited:
            continue
        tile = m.tiles.get(neighbor)
        if tile and tile["known"].get(m.OPPOSITE[d], False):
            count += 1
    return count


def mapped_forward_unvisited_plan(m: TileMap, min_known: int = 2) \
        -> Optional[ExplorationPlan]:
    """地図で開放確認済みの未到達の前方は、一時センサー候補より先に確定する。

    物理的な前進許可ではない。停止距離内の障害物は別途停止・再観測する。
    現在地／隣接側の壁ありは引き続き拒否する。
    """
    with m.lock:
        here = (m.x, m.y)
        tile = m.tiles.get(here)
        direction = m.DIRECTIONS[m.heading]
        if here not in m.visited or tile is None:
            return None
        if not tile["known"].get(direction, False) or tile[direction] != 0:
            return None
        dx, dy = m.STEPS[m.heading]
        target = (m.x + dx, m.y + dy)
        if target in m.visited or target not in set(_map_edges(m, here)):
            return None
        known = _frontier_known_walls(m, target)
        return ExplorationPlan(target, (here, target), known, known < min_known)


def shortest_unvisited_plan(m: TileMap, min_known: int = 2,
                            live_open: Optional[set[int]] = None,
                            blocked_edges: Optional[set[frozenset]] = None) \
        -> Optional[ExplorationPlan]:
    """現在地の未到達1マスを優先し、直進→左→右→Uターンを守る。

    Hough線が見えず壁が未確認でも、生LiDARが実際に開放を裏付けた現在地の
    辺は前方優先候補とする。帰路は既知の通行可能な辺だけでBFSする。
    """
    with m.lock:
        start = (m.x, m.y)
        if start not in m.visited:
            return None
        adjacent_edges = set(_map_edges(m, start, live_open, blocked_edges))
        for relative in (0, 3, 1, 2):
            direction = (m.heading + relative) % 4
            dx, dy = m.STEPS[direction]
            dst = (start[0] + dx, start[1] + dy)
            if dst not in m.visited and dst in adjacent_edges:
                known = _frontier_known_walls(m, dst)
                return ExplorationPlan(dst, (start, dst), known, known < min_known)
        queue = deque([start])
        paths = {start: (start,)}
        targets: list[ExplorationPlan] = []
        while queue:
            current = queue.popleft()
            for nxt in _map_edges(m, current, live_open, blocked_edges):
                if nxt in m.visited:
                    if nxt not in paths:
                        paths[nxt] = paths[current] + (nxt,)
                        queue.append(nxt)
                else:
                    count = _frontier_known_walls(m, nxt)
                    targets.append(ExplorationPlan(nxt, paths[current] + (nxt,),
                                                   count, count < min_known))
        qualified = [t for t in targets if t.known_walls >= min_known]
        options = qualified if qualified else targets
        if not options:
            return None
        preference = {(m.heading + rel) % 4: i for i, rel in
                      enumerate((0, 3, 1, 2))}
        def key(plan):
            first_heading = heading_toward(start, plan.path[1])
            return (len(plan.path), preference[first_heading],
                    -plan.known_walls, plan.target)
        return min(options, key=key)


def shortest_observation_path(m: TileMap, blocked_edges: Optional[set[frozenset]] = None) \
        -> Optional[tuple[tuple[int, int], ...]]:
    """既知の通路だけを通って、未到達方向に未確認辺がある既訪問タイルへ戻る。

    未確認辺そのものは経路に含めない。現在地は直前に再観測済みなので除外。
    """
    with m.lock:
        start = (m.x, m.y)
        queue = deque([start])
        paths = {start: (start,)}
        while queue:
            current = queue.popleft()
            tile = m.tiles.get(current)
            if current != start and tile is not None:
                for d in m.DIRECTIONS:
                    dx, dy = m.DELTAS[d]
                    other = (current[0] + dx, current[1] + dy)
                    if other not in m.visited and not tile["known"].get(d, False):
                        return paths[current]
            for nxt in _map_edges(m, current, blocked_edges=blocked_edges):
                if nxt in m.visited and nxt not in paths:
                    paths[nxt] = paths[current] + (nxt,)
                    queue.append(nxt)
        return None


def enclosed_exploration_complete(m: TileMap, threshold: int = 8) -> bool:
    """到達済み8以上で、形状を問わず全ての境界辺が確認済みの壁で閉じている。"""
    with m.lock:
        if len(m.visited) < threshold:
            return False
        coords = set(m.tiles)
        if not m.visited.issubset(coords):
            return False
        # 開放済みの辺の先に未到達マスがあれば、まだ探索中。
        # 外周が未確認でも完了と判定すると、前方開放時に帰還を始めてしまう。
        if coords != m.visited:
            return False
        for x, y in coords:
            tile = m.tiles[(x, y)]
            for direction in m.DIRECTIONS:
                dx, dy = m.DELTAS[direction]
                neighbor = (x + dx, y + dy)
                if neighbor not in coords:
                    if not tile["known"][direction] or tile[direction] != 1:
                        return False
        return True


def shortest_return_path(m: TileMap, destination: tuple[int, int] = (0, 0),
                         blocked_edges: Optional[set[frozenset]] = None) \
        -> Optional[tuple[tuple[int, int], ...]]:
    """到達済みかつ通行確認済みの隣接タイルだけを通るBFS。"""
    with m.lock:
        start = (m.x, m.y)
        if start not in m.visited or destination not in m.visited:
            return None
        queue = deque([start])
        paths = {start: (start,)}
        while queue:
            current = queue.popleft()
            if current == destination:
                return paths[current]
            for nxt in _map_edges(m, current, blocked_edges=blocked_edges):
                if nxt in m.visited and nxt not in paths:
                    paths[nxt] = paths[current] + (nxt,)
                    queue.append(nxt)
        return None


def heading_toward(start: tuple[int, int], target: tuple[int, int]) -> int:
    delta = (target[0] - start[0], target[1] - start[1])
    if delta not in TileMap.STEPS:
        raise ValueError(f"隣接していない経路: {start} -> {target}")
    return TileMap.STEPS.index(delta)


def distance_to_next_center(m: TileMap, target: tuple[int, int]) -> float:
    """現在の推定位置から次のタイル中心までの必要前進量。"""
    with m.lock:
        direction = heading_toward((m.x, m.y), target)
        dx, dy = m.STEPS[direction]
        remaining = ((target[0]*RUN_DISTANCE_MM - m.position_mm[0])*dx
                     + (target[1]*RUN_DISTANCE_MM - m.position_mm[1])*dy)
        return remaining


def approach_front_before_turn(motors: Motors, lidar: MS200, view: SharedView,
                               stopping: threading.Event, args: argparse.Namespace,
                               initial_clearance: float) -> Optional[float]:
    """同じタイルの前壁まで接近し、実測で停止距離に達した場合だけ移動量を返す。"""
    stop_mm = args.stop_command_cm * 10.0
    max_travel = max(0.0, initial_clearance - stop_mm) + FRONT_APPROACH_EXTRA_MM
    deadline = time.monotonic() + args.turn_timeout_seconds
    previous_left, previous_right = motors.encoder_pair()
    travelled = 0.0
    try:
        while not stopping.is_set():
            current_left, current_right = motors.encoder_pair()
            travelled += (abs(signed_encoder_delta(current_left, previous_left))
                          + abs(signed_encoder_delta(current_right, previous_right))) / 2.0 * (
                              math.pi * WHEEL_DIAMETER_MM / ENCODER_COUNTS)
            previous_left, previous_right = current_left, current_right
            scan = lidar.snapshot()
            check_lidar(lidar, scan)
            frame, _, front, left, right, left_ahead, right_ahead = process_and_render(
                scan, travelled, "BEFORE TURN: APPROACH FRONT")
            view.update(frame)
            if front is None or not math.isfinite(front):
                print("[TURN APPROACH] 前壁を見失ったため旋回せず停止")
                return None
            if front <= stop_mm:
                print(f"[TURN APPROACH] 停止距離へ到達: front={front:.1f}mm")
                motors.stop(force=True)
                final_left, final_right = motors.encoder_pair()
                travelled += (abs(signed_encoder_delta(final_left, previous_left))
                              + abs(signed_encoder_delta(final_right, previous_right))) / 2.0 * (
                                  math.pi * WHEEL_DIAMETER_MM / ENCODER_COUNTS)
                return travelled
            if (travelled >= max_travel or time.monotonic() >= deadline
                    or front > initial_clearance + FRONT_APPROACH_EXTRA_MM):
                print("[TURN APPROACH] 未到達で接近上限。旋回せず停止")
                return None
            distance_error, angle_error, _ = running_wall_guidance(
                scan, left, right, left_ahead, right_ahead)
            correction = int(np.clip(args.steering_sign * (
                KP_WALL_DISTANCE * distance_error + KP_WALL_ANGLE * angle_error),
                -MAX_STEERING, MAX_STEERING))
            motors.drive(*mix_motor_power(NEAR_WALL_POWER, correction))
            stopping.wait(CONTROL_INTERVAL_SECONDS)
        return None
    finally:
        motors.stop(force=True)


def navigation_turn(motors: Motors, lidar: MS200, view: SharedView,
                    stopping: threading.Event, desired_heading: int,
                    args: argparse.Namespace) -> Optional[bool]:
    """True=完了、False=失敗、None=接近後の地図で進路の再選択が必要。"""
    relative = (desired_heading - tile_map.heading) % 4
    if relative == 0:
        return True
    direction, nominal = {1: ("RIGHT", 90), 2: ("LEFT", 180),
                          3: ("LEFT", 90)}[relative]
    reference = lidar.snapshot()
    check_lidar(lidar, reference)
    before_lines = capture_hough_scan(reference)
    stop_mm = args.stop_command_cm * 10.0
    front, _ = front_wall_measurement(before_lines.lines, stop_mm)
    # 地図の「現タイルに壁あり」と走行の「停止距離へ到達」を分離する。
    # 次タイル奥の壁が見えているだけの分岐・探索旋回には適用しない。
    if (front is not None and math.isfinite(front)
            and stop_mm < front <= tile_boundary_clearance_mm(tile_map, 0)):
        print(f"[TURN APPROACH] front={front:.1f}mm > stop={stop_mm:.1f}mm。先に接近")
        tile_map.begin_forward_segment()
        distance = approach_front_before_turn(motors, lidar, view, stopping, args, front)
        if distance is None or stopping.is_set():
            return False
        walls = observe_tile_walls(lidar, view, stopping, front_stop=True)
        if stopping.is_set():
            return False
        tile_map.finish_forward(distance, walls)
        # 接近が5cm未満でも、座標を動かさず壁観測を更新して再探索する。
        tile_map.refresh_wall_observation(walls)
        print("[TURN APPROACH] 接近後の地図を更新。停止失敗ではなく進路を再探索")
        view.set_turn_message("接近完了: 地図更新・進路再探索")
        return None
    # 通常旋回は既存のHough＋エンコーダ処理。終了後は全距離の側壁で補正する。
    coarse, expected_rot, _ = front_wall_turn_plan(
        before_lines.lines, direction, nominal, args.stop_command_cm * 10.0)
    expected = expected_turn_scan(before_lines, expected_rot)
    view.reset_turn()
    view.update_turn_scan("before", before_lines)
    view.update_turn_scan("expected", expected)
    tile_map.begin_turn()
    rotation = turn_in_place(
        motors, lidar, view, 0.0, stopping, direction, coarse,
        args.turn_power,
        args.turn_90_counts * (args.turn_180_scale if nominal == 180 else args.turn_90_scale),
        args.turn_timeout_seconds,
    )
    if stopping.is_set():
        return False
    tile_map.turn(direction, nominal, rotation)
    wait_after_turn(lidar, view, stopping, args.turn_settle_seconds)
    if stopping.is_set():
        return False
    # 同じ誤差へHough追加補正と側壁補正を二重適用しない。
    return align_sidewalls_unlimited(motors, lidar, view, stopping, args, phase="TURN")

# display_manager5.pyだけがGPIO22を物理的に確保する。
ROBOT_BUTTON_SOCKET = Path(os.environ.get("ROBOT_BUTTON_SOCKET", "/dev/shm/robocup_gpio22_press.sock"))


class RobotButtonPressReceiver:
    """UNIX datagramでGPIO22の押下を受け取る。GPIO22ピンは操作しない。"""

    def __init__(self, callback):
        self.callback = callback
        self.stop_event = threading.Event()
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.thread = None
        try:
            ROBOT_BUTTON_SOCKET.parent.mkdir(parents=True, exist_ok=True)
            # 排他ロックを確保したrobot_mapping5だけが起動するため、残骸を除去可能。
            try:
                ROBOT_BUTTON_SOCKET.unlink()
            except FileNotFoundError:
                pass
            self.socket.bind(str(ROBOT_BUTTON_SOCKET))
            os.chmod(ROBOT_BUTTON_SOCKET, 0o660)
            self.socket.settimeout(0.2)
            self.thread = threading.Thread(target=self._listen, name="gpio22-button-ipc", daemon=True)
            self.thread.start()
            print(f"[GPIO22] display_managerからの押下受信を開始: {ROBOT_BUTTON_SOCKET}")
        except BaseException:
            self.socket.close()
            raise

    def _listen(self):
        while not self.stop_event.is_set():
            try:
                message = self.socket.recv(32)
            except socket.timeout:
                continue
            except OSError:
                if not self.stop_event.is_set():
                    print("[GPIO22] ボタン受信ソケットが閉じました")
                return
            if message == b"PRESS":
                try:
                    self.callback()
                except Exception as exc:
                    print(f"[GPIO22] 押下コールバックでエラー: {exc}")

    def close(self):
        self.stop_event.set()
        self.socket.close()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        try:
            ROBOT_BUTTON_SOCKET.unlink()
        except FileNotFoundError:
            pass


def notify_retry_display() -> None:
    """GPIO27で停止した後、display_managerへ2秒のRETRY優先表示を通知。"""
    try:
        RETRY_SIGNAL_PATH.parent.mkdir(parents=True, exist_ok=True)
        tmp = RETRY_SIGNAL_PATH.with_name(RETRY_SIGNAL_PATH.name + f".{os.getpid()}.tmp")
        tmp.write_text(str(time.time_ns()), encoding="ascii")
        os.replace(tmp, RETRY_SIGNAL_PATH)
    except OSError as exc:
        print(f"[RETRY LOGO] 通知失敗（停止動作は継続）: {exc}")


class EmergencyRestartButtons:
    """GPIO27の緊急停止を走行ループより優先し、ソフト再起動を要求する。"""

    def __init__(self):
        try:
            from gpiozero import Button
        except ImportError as exc:
            raise RuntimeError(
                "gpiozeroが必要です。Raspberry Piで .venv/bin/python -m pip install gpiozero "
                "または環境に合わせて apt install python3-gpiozero を実行してください"
            ) from exc
        self.reset_requested = threading.Event()
        self.start_requested = threading.Event()
        self.exit_requested = threading.Event()
        self._last_start_press: Optional[float] = None
        self._lock = threading.RLock()
        self._stopping: Optional[threading.Event] = None
        self._motors: Optional[Motors] = None
        # GPIO27・17のみロボットが所有。GPIO22はdisplay_managerの単一所有。
        self.btn3 = Button(27, pull_up=True, bounce_time=0.05)
        try:
            self.btn4 = Button(17, pull_up=True, bounce_time=0.05)
        except Exception:
            self.btn3.close()
            raise
        self.btn3.when_pressed = self._emergency_press
        self.btn4.when_pressed = self._unused_press
        try:
            self.btn1_receiver = RobotButtonPressReceiver(self._start_press)
        except Exception:
            self.btn3.close()
            self.btn4.close()
            raise
        print("[GPIO] Button1=GPIO22: display_manager経由 / Button3=GPIO27: 緊急停止と再待機 "
              "/ Button4=GPIO17: 予備")

    def _start_press(self) -> None:
        """0.5秒以内の2連打は、始動操作よりも優先して終了する。"""
        now = time.monotonic()
        with self._lock:
            double = (self._last_start_press is not None
                      and now - self._last_start_press <= 0.5)
            self._last_start_press = None if double else now
        if double:
            print("[GPIO] Button1 x2 / 0.5s: 終了・モーター最優先停止")
            self.exit_requested.set()
            self.reset_requested.set()  # 再加速ガードと同じラッチを使用する
            self.start_requested.clear()
            with self._lock:
                stopping, motors = self._stopping, self._motors
            if stopping is not None:
                stopping.set()
            if motors is not None:
                try:
                    motors.emergency_stop()
                except Exception as exc:
                    print(f"[EXIT] モーター停止命令エラー: {exc}")
            tile_map.suspend_updates()
        elif not self.reset_requested.is_set() and not self.btn3.is_pressed:
            self.start_requested.set()
            print("[GPIO] Button 1 pressed: single/ダブル判定待ち")

    @staticmethod
    def _unused_press() -> None:
        print("[GPIO] Button 4 pressed: 未割り当て")

    def _emergency_press(self) -> None:
        # 最初にラッチ。以降、モーターのdrive()は非ゼロ出力を無視する。
        self.reset_requested.set()
        self.start_requested.clear()
        with self._lock:
            stopping = self._stopping
            motors = self._motors
        if stopping is not None:
            stopping.set()
        print("[EMERGENCY] GPIO27 Button3: 全輪停止→再初期化→待機")
        if motors is not None:
            try:
                motors.emergency_stop()
            except Exception as exc:
                print(f"[EMERGENCY] モーターへの停止命令に失敗: {exc}")
                # 通信断でも停止保証はできないため外部モーター電源の遮断が必要。
        # 最優先のモーター停止命令を妨げないよう、その後でマップを凍結する。
        tile_map.suspend_updates()
        notify_retry_display()  # モーター停止命令を優先した後に表示要求
        print("[EMERGENCY] 現在のタイル地図・方位・JSONを変更せず保持")

    def begin_cycle(self, stopping: threading.Event) -> None:
        with self._lock:
            self._stopping = stopping
            self._motors = None
            self.reset_requested.clear()
            self.start_requested.clear()
        self._last_start_press = None

    def attach_motors(self, motors: Motors) -> None:
        with self._lock:
            self._motors = motors
            if self.reset_requested.is_set():
                motors.emergency_stop()

    def detach_motors(self) -> None:
        with self._lock:
            self._motors = None

    def wait_for_start(self, stopping: threading.Event, allow_enter: bool = True) -> bool:
        """待機中は駆動しない。ボタン1またはVS CodeターミナルのEnterで開始。"""
        self.start_requested.clear()  # 走行中に押されたボタン1を持ち越さない
        message = ("Button1(GPIO22)またはEnter" if allow_enter else "Button1(GPIO22)のみ")
        print(f"[STANDBY] モーター停止中。{message}で走行開始。")
        stdin_enabled = allow_enter and sys.stdin is not None and sys.stdin.isatty()
        while not stopping.is_set() and not self.reset_requested.is_set():
            if self.btn3.is_pressed:
                stopping.wait(0.05)
                continue
            if self.start_requested.is_set():
                # ダブル押下が成立する0.5秒間はモーター起動しない。
                with self._lock:
                    first_press = self._last_start_press
                if first_press is not None and time.monotonic() - first_press < 0.5:
                    stopping.wait(0.02)
                    continue
                self.start_requested.clear()
                return not self.exit_requested.is_set()
            if stdin_enabled:
                try:
                    readable, _, _ = select.select([sys.stdin], [], [], 0)
                    if readable:
                        if sys.stdin.readline():
                            return not stopping.is_set() and not self.reset_requested.is_set()
                        stdin_enabled = False  # EOF時はGPIO入力のみ
                except (OSError, ValueError):
                    stdin_enabled = False
            stopping.wait(0.03)
        return False

    def wait_button3_release(self) -> None:
        if self.btn3.is_pressed:
            print("[EMERGENCY] Button3を離すと待機モードで再初期化します")
        while self.btn3.is_pressed:
            time.sleep(0.05)

    def close(self) -> None:
        # 先に受信スレッドを終了してからボタンと制御状態を破棄する。
        self.btn1_receiver.close()
        for button in (self.btn3, self.btn4):
            button.when_pressed = None
            button.close()


def _robot_process_ancestors() -> set[int]:
    """現在の実行を起動した親プロセス群を調べる（VS Code/debugpy対策）。

    debugpy launcher のコマンドラインにも robot_mapping5.py が含まれるが、
    この launcher は同じ実行の親であり、別のロボット制御プロセスではない。
    """
    ignored = {os.getpid()}
    pid = os.getpid()
    for _ in range(64):
        try:
            status = Path(f"/proc/{pid}/status").read_text(encoding="ascii")
            ppid_line = next((line for line in status.splitlines()
                              if line.startswith("PPid:")), None)
            if ppid_line is None:
                break
            parent = int(ppid_line.split()[1])
        except (OSError, ValueError, IndexError):
            break
        if parent <= 1 or parent in ignored:
            break
        ignored.add(parent)
        pid = parent
    return ignored


def _active_robot_processes() -> list[tuple[int, str]]:
    """本当に別に動いている robot_mapping5.py の実行を検出する。"""
    found = []
    proc = Path("/proc")
    if not proc.exists():
        return found
    ignored_pids = _robot_process_ancestors()
    for item in proc.iterdir():
        if not item.name.isdecimal():
            continue
        candidate_pid = int(item.name)
        if candidate_pid in ignored_pids:
            continue
        try:
            arguments = (item / "cmdline").read_bytes().split(b"\0")
            arguments = [os.fsdecode(arg) for arg in arguments if arg]
        except (OSError, ValueError):
            continue
        if not arguments:
            continue
        binary = os.path.basename(arguments[0]).lower()
        if not ("python" in binary or "debugpy" in binary):
            continue
        if any(os.path.basename(arg) == "robot_mapping5.py" for arg in arguments[1:]):
            found.append((candidate_pid, " ".join(arguments)[:200]))
    return found


def _acquire_robot_instance_lock():
    """二重起動を禁止する。debugpyの親プロセスは除外して判定する。"""
    lock_path = "/tmp/robocup_robot_mapping5.lock"
    lock = open(lock_path, "a+", encoding="ascii")
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        others = _active_robot_processes()
        if others:
            details = "; ".join(f"PID={pid}: {cmd}" for pid, cmd in others[:5])
            raise RuntimeError(
                "robot_mapping5.pyが既に実行されています。二重起動はモーター競合の危険があるため停止しました。 "
                + details + "。既存プログラムを安全に停止してから再実行してください。"
            )
        lock.seek(0)
        lock.truncate()
        lock.write(str(os.getpid()) + "\n")
        lock.flush()
    except BaseException:
        lock.close()
        raise
    return lock


def run(args: argparse.Namespace) -> None:
    # プロセス・モーターの二重起動を防止。リスタート時も同じロックを維持。
    instance_lock = _acquire_robot_instance_lock()
    color_camera = None
    try:
        buttons = EmergencyRestartButtons()
        try:
            if args.color_camera is not None:
                color_camera = ColorCamera(args.color_camera, tuple(args.color_roi))
                color_camera.thread.start()
            restarting = False
            while True:
                if restarting:
                    buttons.wait_button3_release()
                stopping = threading.Event()
                buttons.begin_cycle(stopping)
                GOAL_DISPLAY_ACTIVE.clear()
                try:
                    _run_unlocked(args, buttons, stopping, restart_standby=restarting,
                                  color_camera=color_camera)
                except Exception as exc:
                    if not buttons.reset_requested.is_set():
                        raise
                    print(f"[EMERGENCY] 停止要求中の例外は再初期化で回復します: {exc}")
                if buttons.exit_requested.is_set():
                    print("[EXIT] Button1 double press: 終了")
                    break
                if not buttons.reset_requested.is_set():
                    break
                # 同一プロセスでLiDAR・Web・地図・モーターを再生成する。
                # debugpy接続を維持でき、プロセスが二重起動することもない。
                restarting = True
                print("[EMERGENCY] 再初期化してSTANDBYへ移行します")
        finally:
            buttons.close()
    finally:
        if color_camera is not None:
            color_camera.stopping.set()
            if color_camera.thread.ident is not None:
                color_camera.thread.join(timeout=2.0)
        instance_lock.close()


def _run_unlocked(args: argparse.Namespace, buttons: EmergencyRestartButtons,
                  stopping: threading.Event, restart_standby: bool = False,
                  color_camera: Optional[ColorCamera] = None) -> None:
    """停止を起点とした最短frontier探索。再起動時のマップは保持。"""
    global tile_map
    if restart_standby:
        tile_map.suspend_updates()
        print(f"[MAP] GPIO27: {len(tile_map.visited)}到達済みタイルを保持（更新なし）")
    else:
        tile_map = TileMap()
        tile_map.path = Path(args.map_file).expanduser().resolve()
        tile_map.save()
    lidar = MS200(args.lidar_port, args.lidar_baud, args.angle_offset)
    motors: Optional[Motors] = None
    view = SharedView()
    server: Optional[uvicorn.Server] = None
    lcd_thread: Optional[threading.Thread] = None
    previous_sigint, previous_sigterm = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, lambda *_: stopping.set())
    signal.signal(signal.SIGTERM, lambda *_: stopping.set())
    command = "WAIT" if not args.view_only else "VIEW ONLY"
    returning = False
    last_route: Optional[tuple[tuple[int, int], ...]] = None
    current_target: Optional[tuple[int, int]] = None
    target_mm = RUN_DISTANCE_MM
    travelled = 0.0
    total_travelled = 0.0
    segment_no = 0
    last_render = 0.0
    last_control = 0.0
    last_guidance = 0.0
    cached_guidance = (0.0, 0.0, "ANGLE ONLY: WAIT")
    center_retry_count = 0
    front_approach = FrontWallApproach()
    side_filter = SideWallFilter()
    wall_clearance = None
    left_clearance = right_clearance = left_ahead = right_ahead = None
    previous_left = previous_right = 0
    segment_start_scan: Optional[Scan] = None
    start_lateral_error_mm = 0.0
    try:
        server = start_web(view, args.web_port, color_camera)
        if args.lcd_mode != "off":
            lcd_thread = threading.Thread(
                target=lcd_mirror_loop,
                args=(view, stopping, args.lcd_mode, args.lcd_fps, args.lcd_cycle_seconds),
                daemon=True, name="lcd-file-mirror")
            lcd_thread.start()
        lidar.start()
        wait_for_lidar(lidar)
        if stopping.is_set():
            return
        print(f"{PROGRAM_VERSION}: Web全体 / ST7789={args.lcd_mode} / 最短frontier探索")
        if args.view_only:
            while not stopping.is_set():
                render(lidar, view, 0.0, "VIEW ONLY")
                stopping.wait(RENDER_INTERVAL_SECONDS)
            return
        if not buttons.wait_for_start(stopping, allow_enter=not restart_standby):
            return
        if stopping.is_set() or buttons.exit_requested.is_set():
            return
        motors = Motors(args.motor_port, emergency=buttons.reset_requested)
        buttons.attach_motors(motors)
        motors.start()
        if stopping.is_set():
            return
        if restart_standby:
            tile_map.resume_updates()
            print("[MAP] GPIO27後はマップ保持。5cm以上前進して通常停止するまで更新しない")
        else:
            # 元の初回スタート姿勢合わせは維持する。
            # 初回姿勢合わせは保持。タイル末の平行補正は専用処理で行う。
            if not align_before_forward(motors, lidar, view, stopping,
                                        min(ALIGNMENT_POWER, args.turn_power),
                                        args.stop_command_cm * 10.0):
                return
            # スタートタイルだけ0cmの停止観測で作成する。
            observed_since = time.monotonic()
            start_walls = observe_tile_walls(lidar, view, stopping)
            if stopping.is_set():
                return
            colors = color_camera.colors_since(observed_since) if color_camera else ()
            tile_map.finish_forward(0.0, start_walls, camera_colors=colors)
            # 初期タイルの観測直後も車体の中心ずれを推定して次の進行量へ反映。
            initial_scan = lidar.snapshot()
            check_lidar(lidar, initial_scan)
            initial_offsets, initial_center_info = estimate_tile_center_unlimited(initial_scan)
            if adopt_lidar_center_estimate(tile_map, initial_offsets):
                print(f"[CENTER] START座標補正: {initial_center_info}")

        navigation_retryable = False

        def next_step() -> bool:
            """停止位置で再観測して経路を決定。障害物なら別候補へ再探索。"""
            nonlocal returning, last_route, current_target, target_mm
            nonlocal travelled, previous_left, previous_right, segment_start_scan
            nonlocal last_control, last_render, segment_no, command, side_filter
            nonlocal wall_clearance, left_clearance, right_clearance, left_ahead, right_ahead
            nonlocal start_lateral_error_mm, navigation_retryable
            nonlocal center_retry_count, last_guidance, cached_guidance
            nonlocal front_approach
            navigation_retryable = False
            if stopping.is_set():
                return False
            motors.stop(force=True)
            if mapped_forward_unvisited_plan(tile_map) is not None:
                # 帰還フラグや一時禁止辺より、地図に記録済みの未到達前方を優先する。
                if returning:
                    print("[NAV] 地図で前方が未到達・壁なし。帰還を解除して直進探索")
                returning = False
            # 壁で囲まれた条件を満たしていても、新しく見通せる未到達方向があれば
            # 早すぎる帰還判定を取り消して探索を優先する。
            if not returning and enclosed_exploration_complete(tile_map):
                completion_scan = lidar.snapshot()
                check_lidar(lidar, completion_scan)
                completion_live = live_open_directions(completion_scan, tile_map)
                with tile_map.lock:
                    here = (tile_map.x, tile_map.y)
                    outside_frontier = any(
                        (here[0] + tile_map.STEPS[d][0],
                         here[1] + tile_map.STEPS[d][1]) not in tile_map.visited
                        for d in completion_live
                    )
                if outside_frontier:
                    print("[NAV] 現在地から未到達方向を実測。GOAL判定より探索を優先します")
                else:
                    returning = True
            blocked: set[frozenset] = set()
            # 通路手前に実障害物があれば、別方向を試す。無限旋回は避ける。
            for attempt in range(8):
                if stopping.is_set():
                    return False
                fresh = lidar.snapshot()
                check_lidar(lidar, fresh)
                # 再観測は壁だけ更新。未知→開放を反映し、Noneで既知情報を消さない。
                confirmed_walls = observe_tile_walls(lidar, view, stopping)
                if stopping.is_set():
                    return False
                tile_map.refresh_wall_observation(confirmed_walls)
                fresh = lidar.snapshot()
                check_lidar(lidar, fresh)
                with tile_map.lock:
                    current_tile = tile_map.tiles.get((tile_map.x, tile_map.y), {})
                    navigation_walls = {}
                    for relative in range(4):
                        direction = tile_map.DIRECTIONS[(tile_map.heading + relative) % 4]
                        navigation_walls[relative] = (current_tile[direction]
                            if current_tile.get("known", {}).get(direction, False)
                            else confirmed_walls.get(relative))
                live, live_blocked = navigation_directions(fresh, tile_map, walls=navigation_walls)
                forward_plan = mapped_forward_unvisited_plan(tile_map)
                with tile_map.lock:
                    here = (tile_map.x, tile_map.y)
                    front_delta = tile_map.STEPS[tile_map.heading]
                    front_target = (here[0] + front_delta[0], here[1] + front_delta[1])
                    front_direction = tile_map.DIRECTIONS[tile_map.heading]
                    local_tile = tile_map.tiles.get(here, {})
                    print(f"[NAV FRONT] here={here}, heading={front_direction}, "
                          f"front={front_target}, visited={front_target in tile_map.visited}, "
                          f"known={local_tile.get('known', {}).get(front_direction, False)}, "
                          f"wall={local_tile.get(front_direction)}, "
                          f"mapped_priority={forward_plan is not None}")
                    for d in live_blocked:
                        if forward_plan is not None and d == tile_map.heading:
                            # 1辺分先の壁や一時ノイズで、確定済みの前方を落とさない。
                            # 衝突防止は前進直前と走行中の停止距離判定で行う。
                            continue
                        dx, dy = tile_map.STEPS[d]
                        blocked.add(_edge_key(here, (here[0] + dx, here[1] + dy)))
                    if forward_plan is not None:
                        blocked.discard(_edge_key(here, forward_plan.target))
                if forward_plan is not None:
                    returning = False
                if returning:
                    route = shortest_return_path(tile_map, blocked_edges=blocked)
                    if route is None:
                        command = "RETURN: WAIT PATH"
                        navigation_retryable = True
                        print("[NAV] 帰還経路が未確定。停止してLiDAR再観測を待ちます")
                        return False
                    if len(route) == 1:
                        print("[GOAL] (0,0)到着。モーター停止・goal.png表示・待機")
                        motors.stop(force=True)
                        motors.close()
                        buttons.detach_motors()
                        tile_map.suspend_updates()
                        publish_goal_lcd()
                        command = "GOAL / STANDBY"
                        while not stopping.is_set() and not buttons.exit_requested.is_set():
                            stopping.wait(0.05)
                        return False
                    decision = route
                    rule = "RETURN ORIGIN"
                else:
                    plan = forward_plan or shortest_unvisited_plan(
                        tile_map, min_known=2, live_open=live, blocked_edges=blocked)
                    if plan is None:
                        observation_route = shortest_observation_path(tile_map, blocked)
                        if observation_route is None:
                            command = "NAV: WAIT FRONTIER (UNKNOWN EDGES)"
                            navigation_retryable = True
                            print(f"[NAV] 未到達への開放辺・再観測地点への既知通路なし: "
                                  f"現在=({tile_map.x},{tile_map.y}), "
                                  f"到達={len(tile_map.visited)}。未知の辺へは進まず再観測")
                            return False
                        decision = observation_route
                        rule = "REOBSERVE UNKNOWN EDGE"
                    else:
                        decision = plan.path
                        rule = ("MAPPED FRONT UNVISITED" if forward_plan is not None else
                                "BOOTSTRAP 1-WALL" if plan.bootstrap else "FRONTIER >=2 WALLS")
                last_route = decision
                current_target = decision[1]
                desired = heading_toward(decision[0], decision[1])
                # BFSの経路全体ではなく、直近の1辺が壁を越えないことを旋回前にも確認。
                with tile_map.lock:
                    edge_allowed = current_target in set(_map_edges(
                        tile_map, (tile_map.x, tile_map.y), live, blocked))
                if not edge_allowed:
                    blocked.add(_edge_key(decision[0], current_target))
                    continue
                print(f"[NAV] {rule}: target={decision[-1]}, shortest={len(decision)-1}辺, "
                      f"next={current_target}, visited={len(tile_map.visited)}")
                turn_result = navigation_turn(motors, lidar, view, stopping, desired, args)
                if turn_result is None:
                    # 接近前の経路と一時禁止辺は破棄し、最新地図から次の動作を選ぶ。
                    blocked.clear()
                    current_target = None
                    last_route = None
                    command = "NAV: REPLAN AFTER FRONT APPROACH"
                    continue
                if not turn_result:
                    command = "STOP: TURN ERROR"
                    print("[NAV] 旋回制御が失敗しました。走行を停止します")
                    return False
                if stopping.is_set():
                    return False
                # 旋回後の方角で実障害物を再評価。通行不可ならマップを書き換えず候補を除外。
                fresh = lidar.snapshot()
                check_lidar(lidar, fresh)
                lines = process_pipeline(fresh.points[:, :2]).lines
                clear = front_wall_clearance_mm(lines)
                _, fresh_blocked = navigation_directions(fresh, tile_map)
                immediate_obstacle = (
                    (clear is not None and clear <= args.stop_command_cm * 10.0)
                    or raw_corridor_has_obstacle(
                        fresh, 0.0, ROBOT_RADIUS_MM + args.stop_command_cm * 10.0))
                if forward_plan is not None and immediate_obstacle:
                    # 地図と実測が矛盾しても勝手に左折/Uターンせず、停止して再観測。
                    command = "NAV: MAPPED FRONT WAIT OBSTACLE"
                    navigation_retryable = True
                    print(f"[NAV] 地図は前方未到達・壁なしだが停止距離内に障害物: "
                          f"front={clear} mm。別方向へ曲がらず停止・再観測")
                    return False
                if forward_plan is None and (
                        tile_map.heading in fresh_blocked or immediate_obstacle):
                    edge = _edge_key((tile_map.x, tile_map.y), current_target)
                    blocked.add(edge)
                    print(f"[NAV] 正面障害物を検出。{current_target}を一時除外して再探索 (attempt={attempt+1})")
                    continue
                target_mm = distance_to_next_center(tile_map, current_target)
                if 0.0 < target_mm < MAP_MIN_ADVANCE_MM:
                    # 位置推定で目的タイル中心の直前に来た場合に、<5cm判定で
                    # 無限待機するのを避ける。停止時5cm更新条件は維持。
                    print(f"[NAV] 残距離{target_mm:.1f}mmを最小走行{MAP_MIN_ADVANCE_MM:.0f}mmに補正")
                    target_mm = MAP_MIN_ADVANCE_MM
                if not MAP_MIN_ADVANCE_MM <= target_mm <= RUN_DISTANCE_MM * 1.6:
                    command = "STOP: BAD TARGET DISTANCE"
                    print(f"[NAV] 地図位置と次タイル距離が不一致 ({target_mm:.1f}mm) 。安全停止")
                    return False
                # マップ上では開放済みでも実スキャンが近距離物体を示すなら進まない。
                # 単なるHough線不足だけで戻ったり無限待機したりしない。
                start_lateral_error_mm = float(np.clip(
                    next_heading_lateral_offset_mm(tile_map, desired),
                    -CENTERING_LATERAL_MAX_STEER_MM, CENTERING_LATERAL_MAX_STEER_MM))
                if abs(start_lateral_error_mm) >= CENTERING_TOLERANCE_MM:
                    print(f"[CENTER] 次の進行方向={desired}: 横ずれ推定{start_lateral_error_mm:+.1f}mm。遠壁時は距離操舵へ使いません")
                segment_no += 1
                center_retry_count = 0
                front_approach = FrontWallApproach()
                last_guidance = 0.0
                cached_guidance = (0.0, 0.0, "ANGLE ONLY: WAIT")
                travelled = 0.0
                tile_map.begin_forward_segment()
                segment_start_scan = lidar.snapshot()
                previous_left, previous_right = motors.encoder_pair()
                side_filter = SideWallFilter()
                wall_clearance = left_clearance = right_clearance = left_ahead = right_ahead = None
                last_control = time.monotonic()
                last_render = 0.0
                command = f"NAV {rule} S{segment_no} to {current_target}"
                motors.drive(BASE_POWER, BASE_POWER)
                return True
            navigation_retryable = True
            command = "NAV: WAIT CLEAR ROUTE"
            print("[NAV] 全候補で障害物を検出。停止状態で次回スキャンから再探索")
            return False

        def wait_for_route() -> bool:
            """経路情報不足で固定待機せず、停止状態で周期的に再探索。"""
            nonlocal command
            next_try = 0.0
            while not stopping.is_set() and not buttons.exit_requested.is_set():
                motors.stop(force=True)
                if navigation_retryable and time.monotonic() >= next_try:
                    next_try = time.monotonic() + 1.0
                    if next_step():
                        return True
                render(lidar, view, 0.0, command)
                stopping.wait(0.15)
            return False

        if not next_step() and not wait_for_route():
            return

        while not stopping.is_set():
            now = time.monotonic()
            scan = lidar.snapshot()
            check_lidar(lidar, scan)
            if now - last_render >= RENDER_INTERVAL_SECONDS:
                (frame, _, wall_clearance, raw_left, raw_right,
                 raw_left_ahead, raw_right_ahead) = process_and_render(scan, travelled, command)
                (left_clearance, right_clearance, left_ahead, right_ahead) = side_filter.update(
                    raw_left, raw_right, raw_left_ahead, raw_right_ahead, now)
                view.update(frame)
                last_render = now
            if now - last_control < CONTROL_INTERVAL_SECONDS:
                stopping.wait(0.01)
                continue
            if stopping.is_set():
                break
            current_left, current_right = motors.encoder_pair()
            dl = abs(signed_encoder_delta(current_left, previous_left))
            dr = abs(signed_encoder_delta(current_right, previous_right))
            previous_left, previous_right = current_left, current_right
            step_mm = ((dl + dr) / 2.0) * (math.pi * WHEEL_DIAMETER_MM / ENCODER_COUNTS)
            travelled += step_mm
            total_travelled += step_mm
            last_control = now
            at_front_wall = wall_clearance is not None and wall_clearance <= args.stop_command_cm * 10.0
            at_target = front_approach.distance_stop(
                travelled, target_mm, wall_clearance, args.stop_command_cm * 10.0)
            if at_front_wall or at_target:
                motors.stop(force=True)
                observed_since = time.monotonic()
                if stopping.is_set():
                    break
                # 停止後の最終エンコーダ差分を含める。
                final_left, final_right = motors.encoder_pair()
                travelled += (abs(signed_encoder_delta(final_left, previous_left)) +
                              abs(signed_encoder_delta(final_right, previous_right))) / 2.0 * (
                                  math.pi * WHEEL_DIAMETER_MM / ENCODER_COUNTS)
                command = "STOP: FRONT WALL" if at_front_wall else "STOP: TILE CENTER"
                if front_approach.limit_mm is not None and not at_front_wall:
                    command = "STOP: FRONT APPROACH END"
                walls = observe_tile_walls(lidar, view, stopping, front_stop=at_front_wall)
                if stopping.is_set():
                    break
                # 停止時だけマップ更新。5cm未満ならfinish_forwardが何もしない。
                # 経路はエンコーダ中心で、LiDAR位置補正は信頼性検査を通した場合だけ。
                lidar_motion = estimate_forward_motion(segment_start_scan, lidar.snapshot(), travelled)
                lidar_distance = None if lidar_motion is None else lidar_motion[0]
                colors = color_camera.colors_since(observed_since) if color_camera else ()
                tile_map.finish_forward(travelled, walls, lidar_distance, camera_colors=colors)
                print(f"[NAV] STOP at {tile_map.x},{tile_map.y} after {travelled:.1f}mm")
                # マップは停止時だけ更新。角度補正はその後の独立した
                # 全距離LiDAR測定で行い、走行中の制御には一切混ぜない。
                if travelled >= MAP_MIN_ADVANCE_MM:
                    if not align_tile_end_unlimited(motors, lidar, view, stopping, args):
                        break
                    if stopping.is_set():
                        break
                    # 停止後の全距離LiDARでタイル中心からの偏差を検証。
                    # 壁2組が見えない軸では修正しない（推測で走らせない）。
                    center_scan = lidar.snapshot()
                    check_lidar(lidar, center_scan)
                    offsets, center_info = estimate_tile_center_unlimited(center_scan)
                    if adopt_lidar_center_estimate(tile_map, offsets):
                        print(f"[CENTER] 位置推定を修正: {center_info}")
                    else:
                        print(f"[CENTER] 壁不足/不整合。エンコーダ位置維持: {center_info}")
                # タイル境界を越えただけで中心到達とみなさない。バンプ空転で
                # LiDAR実移動が不足したときは、同じ目標中心まで低速で補完する。
                if (ENABLE_EXPERIMENTAL_BUMP_RECOVERY and not at_front_wall
                        and lidar_distance is not None and current_target is not None):
                    remaining = remaining_to_requested_center_mm(tile_map, current_target)
                    if remaining >= CENTER_RETRY_TOLERANCE_MM:
                        if center_retry_count >= CENTER_RETRY_MAX:
                            print("[CENTER RETRY] 補完上限。バンプ等で到達できないため停止します")
                            break
                        fresh = lidar.snapshot()
                        check_lidar(lidar, fresh)
                        fresh_front = front_wall_clearance_mm(capture_hough_scan(fresh).lines)
                        if fresh_front is None or fresh_front > args.stop_command_cm * 10.0:
                            center_retry_count += 1
                            target_mm = min(remaining, RUN_DISTANCE_MM * 1.6)
                            travelled = 0.0
                            front_approach = FrontWallApproach()
                            tile_map.begin_forward_segment()
                            segment_start_scan = fresh
                            previous_left, previous_right = motors.encoder_pair()
                            side_filter = SideWallFilter()
                            wall_clearance = left_clearance = right_clearance = left_ahead = right_ahead = None
                            last_guidance = last_render = 0.0
                            last_control = time.monotonic()
                            command = f"CENTER RETRY {center_retry_count}: {remaining:.0f}mm"
                            print(f"[CENTER RETRY] 同じ目標{current_target}まで残り{remaining:.1f}mmを補完")
                            motors.drive(NEAR_WALL_POWER, NEAR_WALL_POWER)
                            continue
                travelled = 0.0
                if stopping.is_set():
                    break
                # 進行できなかった場合、同じ壁へ繰り返し突っ込まない。
                # 前壁停止でも目標タイル到達でも、次の一辺を再探索する。
                # 未確認のときは停止したままLiDARを定期的に再評価する。
                if not next_step() and not wait_for_route():
                    break
                continue
            if now - last_guidance >= DISTANT_GUIDANCE_INTERVAL_SECONDS:
                cached_guidance = running_wall_guidance(
                    scan, left_clearance, right_clearance, left_ahead, right_ahead)
                last_guidance = now
            distance_error, angle_error, guidance = cached_guidance
            base = NEAR_WALL_POWER if (wall_clearance is not None and
                                       wall_clearance <= SLOWDOWN_CLEARANCE_MM) else BASE_POWER
            if center_retry_count:
                base = min(base, NEAR_WALL_POWER)
            if (abs(distance_error) >= ALIGN_DISTANCE_THRESHOLD_MM or
                    abs(angle_error) >= ALIGN_ANGLE_THRESHOLD_DEG):
                base = min(base, ALIGN_BASE_POWER)
            correction = int(np.clip(args.steering_sign *
                                     (KP_WALL_DISTANCE * distance_error +
                                      KP_WALL_ANGLE * angle_error),
                                     -MAX_STEERING, MAX_STEERING))
            left, right = mix_motor_power(base, correction)
            motors.drive(left, right)
            command = f"{guidance} NAV S{segment_no} L={left} R={right}"
            stopping.wait(0.015)
    finally:
        stopping.set()
        try:
            if motors is not None:
                motors.close()
        finally:
            buttons.detach_motors()
            try:
                lidar.stop()
            finally:
                if server is not None:
                    server.should_exit = True
                    thread = getattr(server, "robot_thread", None)
                    if thread is not None:
                        thread.join(timeout=3.0)
                if lcd_thread is not None:
                    lcd_thread.join(timeout=2.0)
                signal.signal(signal.SIGINT, previous_sigint)
                signal.signal(signal.SIGTERM, previous_sigterm)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="正面壁で進路を決め、低速旋回しながら連続走行する"
    )
    parser.add_argument("--map-file", default="tile_map.json",
                        help="タイル情報JSONの保存先（起動ごとに新規マップ）")
    parser.add_argument("--lidar-port", default="/dev/ttyAMA0")
    parser.add_argument("--lidar-baud", type=int, default=230400)
    parser.add_argument("--motor-port", default="/dev/ttyUSB0")
    parser.add_argument("--angle-offset", type=float, default=0.0)
    parser.add_argument("--web-port", type=int, default=8000,
                        help="Web開始ポート。使用中なら次の空きポートへ自動切替（既定8000）")
    camera_group = parser.add_mutually_exclusive_group()
    camera_group.add_argument("--color-camera", metavar="DEVICE", default=DEFAULT_COLOR_CAMERA,
                              help=f"赤・黄・緑の検出用FPVカメラ（既定 {DEFAULT_COLOR_CAMERA}）")
    camera_group.add_argument("--no-color-camera", dest="color_camera", action="store_const", const=None,
                              help="カメラの映像取得と色検出を無効にする")
    parser.add_argument("--color-roi", type=float, nargs=4, default=(0, 0, 1, 1),
                        metavar=("X1", "Y1", "X2", "Y2"), help="色検出範囲を画像内の0〜1で指定")
    # Normal startup initializes motors. --view-only is retained as an explicit safe override.
    drive_group = parser.add_mutually_exclusive_group()
    drive_group.add_argument("--drive", action="store_true",
                             help="互換用オプション（通常起動でもモーター有効）")
    drive_group.add_argument("--view-only", action="store_true",
                             help="明示的にモーターを無効化し、LiDAR/Web/ST7789表示のみ実行")
    parser.add_argument("--lcd-mode", choices=("web", "lidar", "map", "turn", "cycle", "off"),
                        default="map", help="ST7789は標準でマップのみ表示")
    parser.add_argument("--lcd-fps", type=float, default=2.0)
    parser.add_argument("--lcd-cycle-seconds", type=float, default=4.0)
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
    if (args.color_camera is not None and not args.color_camera.startswith("/dev/")
            and not args.color_camera.isdecimal()):
        parser.error("--color-camera は /dev/ で始まるデバイス名またはカメラ番号にしてください")
    x1, y1, x2, y2 = args.color_roi
    if not (0 <= x1 < x2 <= 1 and 0 <= y1 < y2 <= 1):
        parser.error("--color-roi は 0 ≤ X1 < X2 ≤ 1、0 ≤ Y1 < Y2 ≤ 1 にしてください")
    if not 1 <= args.web_port <= 65535:
        parser.error("--web-port は1〜65535にしてください")
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
    if not math.isfinite(args.lcd_fps) or not 0.2 <= args.lcd_fps <= 5.0:
        parser.error("--lcd-fps は0.2〜5.0にしてください")
    if not math.isfinite(args.lcd_cycle_seconds) or args.lcd_cycle_seconds <= 0:
        parser.error("--lcd-cycle-seconds は有限の正の値にしてください")
    args.drive = not args.view_only  # Motor control is enabled by default.
    return args

if __name__ == "__main__":
    try:
        run(parse_args())
    except KeyboardInterrupt:
        pass
