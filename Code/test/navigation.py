"""LiDAR点群から壁を抽出し、安全な走行指令を決める層。

このモジュールはGPIOやモーターを直接操作しない。`Navigator.update()` が返す
MotionCommandを、実機固有のモータードライバー層で実行する。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time

import cv2
import numpy as np


class MotionCommand(str, Enum):
    STOP = "stop"
    FORWARD = "forward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"


@dataclass(frozen=True, slots=True)
class Pose:
    x_mm: float
    y_mm: float
    heading_deg: float
    travelled_mm: float


@dataclass(frozen=True, slots=True)
class Line:
    x1_mm: float
    y1_mm: float
    x2_mm: float
    y2_mm: float

    @property
    def length_mm(self) -> float:
        return math.hypot(self.x2_mm - self.x1_mm, self.y2_mm - self.y1_mm)

    @property
    def parallel_error_deg(self) -> float:
        """進行方向(+Y)に対する線の傾き。平行なら0度。"""
        angle = math.degrees(
            math.atan2(self.x2_mm - self.x1_mm, self.y2_mm - self.y1_mm)
        )
        while angle > 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
        return angle


@dataclass(frozen=True, slots=True)
class Perception:
    lines: tuple[Line, ...]
    front_distance_mm: float | None
    front_open: bool
    left_open: bool
    right_open: bool
    parallel_error_deg: float | None
    morphology_image: np.ndarray = field(repr=False, compare=False)


@dataclass(slots=True)
class NavigationConfig:
    image_size_px: int = 600
    range_mm: float = 500.0
    robot_half_width_mm: float = 100.0
    stop_distance_mm: float = 50.0
    move_distance_mm: float = 300.0
    opening_check_distance_mm: float = 300.0
    alignment_tolerance_deg: float = 3.0
    turn_tolerance_deg: float = 3.0
    settle_seconds: float = 0.5
    cell_size_mm: float = 300.0


class LineDetector:
    def __init__(self, config: NavigationConfig | None = None) -> None:
        self.config = config or NavigationConfig()

    def detect(self, points: list[tuple[float, float]]) -> Perception:
        cfg = self.config
        size = cfg.image_size_px
        center = size // 2
        scale = size / (2.0 * cfg.range_mm)

        # 表示用画像とは分離し、グリッド線をHough変換へ入れない。
        binary = np.zeros((size, size), dtype=np.uint8)
        for x_mm, y_mm in points:
            px = round(center + x_mm * scale)
            py = round(center - y_mm * scale)
            if 0 <= px < size and 0 <= py < size:
                cv2.circle(binary, (px, py), 3, 255, -1)

        # 指定された収縮処理。点が完全に消えないよう、入力点を半径3で描く。
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        eroded = cv2.erode(binary, kernel, iterations=1)
        detected = cv2.HoughLinesP(
            eroded,
            rho=1,
            theta=np.pi / 180.0,
            threshold=15,
            minLineLength=20,
            maxLineGap=18,
        )

        lines: list[Line] = []
        if detected is not None:
            for raw in detected[:, 0]:
                x1, y1, x2, y2 = (int(value) for value in raw)
                line = Line(
                    (x1 - center) / scale,
                    (center - y1) / scale,
                    (x2 - center) / scale,
                    (center - y2) / scale,
                )
                if line.length_mm >= 35.0:
                    lines.append(line)

        front = self._direction_distance(lines, "front")
        left = self._direction_distance(lines, "left")
        right = self._direction_distance(lines, "right")
        alignment = self._parallel_error(lines)
        return Perception(
            lines=tuple(lines),
            front_distance_mm=front,
            front_open=front is None or front > cfg.opening_check_distance_mm,
            left_open=left is None or left > cfg.opening_check_distance_mm,
            right_open=right is None or right > cfg.opening_check_distance_mm,
            parallel_error_deg=alignment,
            morphology_image=eroded,
        )

    def _samples(self, line: Line) -> list[tuple[float, float]]:
        count = max(2, math.ceil(line.length_mm / 20.0))
        return [
            (
                line.x1_mm + (line.x2_mm - line.x1_mm) * index / count,
                line.y1_mm + (line.y2_mm - line.y1_mm) * index / count,
            )
            for index in range(count + 1)
        ]

    def _direction_distance(self, lines: list[Line], direction: str) -> float | None:
        cfg = self.config
        distances: list[float] = []
        for line in lines:
            for x_mm, y_mm in self._samples(line):
                if direction == "front" and y_mm > 0 and abs(x_mm) <= cfg.robot_half_width_mm:
                    distances.append(y_mm)
                elif direction == "left" and x_mm < 0 and abs(y_mm) <= cfg.robot_half_width_mm:
                    distances.append(-x_mm)
                elif direction == "right" and x_mm > 0 and abs(y_mm) <= cfg.robot_half_width_mm:
                    distances.append(x_mm)
        return min(distances) if distances else None

    @staticmethod
    def _parallel_error(lines: list[Line]) -> float | None:
        # 進行方向とおおむね平行な長い線だけを姿勢補正に使用する。
        candidates = [
            line.parallel_error_deg
            for line in lines
            if line.length_mm >= 100.0 and abs(line.parallel_error_deg) <= 30.0
        ]
        return float(np.median(candidates)) if candidates else None


class Navigator:
    """30 cm単位で停止・観測し、前→左→右の順に進路を選ぶ。"""

    def __init__(self, config: NavigationConfig | None = None) -> None:
        self.config = config or NavigationConfig()
        self.state = "moving"
        self.stop_started = 0.0
        self.segment_start_mm = 0.0
        self.target_heading_deg: float | None = None
        self.map_marks: dict[tuple[int, int], int] = {}

    def update(self, perception: Perception, pose: Pose, now: float | None = None) -> MotionCommand:
        now = time.monotonic() if now is None else now
        cfg = self.config

        if self.state == "turning":
            assert self.target_heading_deg is not None
            error = self._angle_error(self.target_heading_deg, pose.heading_deg)
            if abs(error) <= cfg.turn_tolerance_deg:
                self.state = "moving"
                self.segment_start_mm = pose.travelled_mm
                self.target_heading_deg = None
                return MotionCommand.STOP
            return MotionCommand.TURN_LEFT if error > 0 else MotionCommand.TURN_RIGHT

        if self.state == "stopped":
            if now - self.stop_started < cfg.settle_seconds:
                return MotionCommand.STOP
            self._mark_directions(perception, pose)
            if perception.front_open:
                self.state = "moving"
                self.segment_start_mm = pose.travelled_mm
                return MotionCommand.FORWARD
            if perception.left_open:
                self.state = "turning"
                self.target_heading_deg = (pose.heading_deg + 90.0) % 360.0
                return MotionCommand.TURN_LEFT
            if perception.right_open:
                self.state = "turning"
                self.target_heading_deg = (pose.heading_deg - 90.0) % 360.0
                return MotionCommand.TURN_RIGHT
            return MotionCommand.STOP

        travelled = pose.travelled_mm - self.segment_start_mm
        too_close = (
            perception.front_distance_mm is not None
            and perception.front_distance_mm <= cfg.stop_distance_mm
        )
        if too_close or travelled >= cfg.move_distance_mm:
            self.state = "stopped"
            self.stop_started = now
            return MotionCommand.STOP

        # 側壁が進行方向と平行でなければ、まず車体角度を微調整する。
        error = perception.parallel_error_deg
        if error is not None and abs(error) > cfg.alignment_tolerance_deg:
            return MotionCommand.TURN_LEFT if error > 0 else MotionCommand.TURN_RIGHT
        return MotionCommand.FORWARD

    def _mark_directions(self, perception: Perception, pose: Pose) -> None:
        """隣接セルを(x, y, n)として保持。open=0、blocked=1。"""
        cell_x = round(pose.x_mm / self.config.cell_size_mm)
        cell_y = round(pose.y_mm / self.config.cell_size_mm)
        heading = round(pose.heading_deg / 90.0) % 4
        # heading 0:+Y, 1:-X, 2:-Y, 3:+X
        vectors = ((0, 1), (-1, 0), (0, -1), (1, 0))
        values = (
            (heading, perception.front_open),
            ((heading + 1) % 4, perception.left_open),
            ((heading - 1) % 4, perception.right_open),
        )
        for direction, is_open in values:
            dx, dy = vectors[direction]
            self.map_marks[(cell_x + dx, cell_y + dy)] = 0 if is_open else 1

    def marks_xyz(self) -> list[tuple[int, int, int]]:
        return [(x, y, value) for (x, y), value in sorted(self.map_marks.items())]

    @staticmethod
    def _angle_error(target_deg: float, current_deg: float) -> float:
        return (target_deg - current_deg + 180.0) % 360.0 - 180.0
