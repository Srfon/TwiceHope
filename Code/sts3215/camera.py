#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RoboCup Pi 5 USB camera: single V4L2 owner, color / H-S-U recognition, live frames.

Only this service opens /dev/video0. robot_mapping5.py consumes snapshots and
shared frames; no second OpenCV, FFmpeg or ustreamer device opener is needed.
H/S/U OCR is observational and MUST NOT directly trigger kit release.
"""
from __future__ import annotations

import asyncio
from collections import deque
from fractions import Fraction
import re
import shutil
import threading
import time
from typing import Optional

import cv2
import numpy as np

COLOR_HSV_BANDS = {
    "red": ((0, 12), (170, 179)),
    "yellow": ((18, 35),),
    "green": ((38, 85),),
}
COLOR_BGR = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0)}
COLOR_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
COLOR_INTERVAL_SECONDS = 0.10
OCR_INTERVAL_SECONDS = 0.75


def analyze_color_frame(frame: np.ndarray, *,
                        min_area: int = 350,
                        min_saturation: int = 85,
                        min_value: int = 60,
                        roi: tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0)) -> list[dict]:
    if frame is None or frame.size == 0:
        return []
    height, width = frame.shape[:2]
    rx1, ry1, rx2, ry2 = roi
    x1, y1, x2, y2 = int(width * rx1), int(height * ry1), int(width * rx2), int(height * ry2)
    if x2 <= x1 or y2 <= y1:
        raise ValueError("Invalid color ROI")
    hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
    results: list[dict] = []
    for color, ranges in COLOR_HSV_BANDS.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(
                hsv, (low, min_saturation, min_value), (high, 255, 255)))
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
        results.append({"color": color, "area": int(round(area)),
                        "bbox": [bx + x1, by + y1, bw, bh],
                        "center": [bx + x1 + bw // 2, by + y1 + bh // 2]})
    return sorted(results, key=lambda d: d["area"], reverse=True)


def analyze_hsu_frame(frame: np.ndarray,
                      roi: tuple[float, float, float, float]) -> list[dict]:
    """Conservative experimental OCR. Can miss characters; validation required."""
    import pytesseract
    from pytesseract import Output
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = (int(w * roi[0]), int(h * roi[1]),
                      int(w * roi[2]), int(h * roi[3]))
    gray = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.equalizeHist(gray)
    data = pytesseract.image_to_data(
        gray, output_type=Output.DICT,
        config="--psm 11 -c tessedit_char_whitelist=HSU",
        timeout=0.6)
    found = []
    for i, token in enumerate(data["text"]):
        letter = token.strip().upper()
        if not re.fullmatch(r"[HSU]", letter):
            continue
        try:
            confidence = float(data["conf"][i])
        except (ValueError, TypeError):
            continue
        if confidence < 70:
            continue
        bx = int(data["left"][i]) + x1
        by = int(data["top"][i]) + y1
        bw = int(data["width"][i])
        bh = int(data["height"][i])
        if bw < 10 or bh < 14:
            continue
        found.append({"letter": letter, "confidence": round(confidence, 1),
                      "bbox": [bx, by, bw, bh]})
    return found


def open_color_camera(source: str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    device = source if source.startswith("/dev/") else int(source)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError(f"USBカメラを開けません: {source}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


class ColorCamera:
    """Backward-compatible robot_mapping5 camera interface and shared frame hub."""

    def __init__(self, source: str, roi: tuple[float, float, float, float],
                 preview_fps: int = 30, ocr_enabled: bool = True):
        self.source = source
        self.roi = roi
        self.preview_fps = max(1, min(int(preview_fps), 60))
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.history: deque[tuple[float, set[str]]] = deque(maxlen=7)
        self.letter_history: deque[tuple[float, set[str]]] = deque(maxlen=5)
        self.message = "カメラ起動中"
        self.frame: Optional[bytes] = None
        self.raw_frame: Optional[np.ndarray] = None
        self.revision = 0
        self.frame_stamp = 0.0
        self.captured_fps = 0.0
        self.color_detections: list[dict] = []
        self.letter_detections: list[dict] = []
        self.ocr_enabled = bool(ocr_enabled and shutil.which("tesseract"))
        self.ocr_message = "OCR待機中" if self.ocr_enabled else "OCR無効（tesseract未導入または--no-ocr）"
        self.thread = threading.Thread(target=self._capture, name="usb-camera-capture", daemon=True)
        self.color_thread = threading.Thread(target=self._analyze_colors, name="usb-camera-color", daemon=True)
        self.ocr_thread = threading.Thread(target=self._analyze_letters, name="usb-camera-ocr", daemon=True)

    def start(self) -> None:
        self.thread.start()
        self.color_thread.start()
        if self.ocr_enabled:
            self.ocr_thread.start()

    def stop(self) -> None:
        self.stopping.set()
        for thread in (self.thread, self.color_thread, self.ocr_thread):
            if thread.ident is not None:
                thread.join(timeout=2.0)

    def _stable_colors(self, since: float = 0.0) -> tuple[str, ...]:
        cutoff = max(since, time.monotonic() - 1.0)
        samples = [items for stamp, items in self.history if stamp >= cutoff]
        votes = {c: sum(c in items for items in samples) for c in COLOR_HSV_BANDS}
        return tuple(sorted((c for c, n in votes.items() if n >= 5),
                            key=votes.__getitem__, reverse=True))

    def _stable_letters(self, since: float = 0.0) -> tuple[str, ...]:
        cutoff = max(since, time.monotonic() - 3.0)
        samples = [items for stamp, items in self.letter_history if stamp >= cutoff]
        votes = {c: sum(c in items for items in samples) for c in "HSU"}
        return tuple(sorted((c for c, n in votes.items() if n >= 3),
                            key=votes.__getitem__, reverse=True))

    def colors_since(self, since: float) -> tuple[str, ...]:
        with self.lock:
            return self._stable_colors(since)

    def letters_since(self, since: float) -> tuple[str, ...]:
        with self.lock:
            return self._stable_letters(since)

    def get_raw_frame(self) -> Optional[np.ndarray]:
        with self.lock:
            if self.raw_frame is None or time.monotonic() - self.frame_stamp > 1.0:
                return None
            return self.raw_frame.copy()

    def snapshot(self) -> dict:
        with self.lock:
            fresh = self.frame is not None and time.monotonic() - self.frame_stamp <= 1.0
            return {
                "message": self.message if fresh or self.frame is None else "カメラ映像が停止中",
                "colors": self._stable_colors(),
                "letters": self._stable_letters(),
                "ocr_status": self.ocr_message,
                "fps": round(self.captured_fps, 1),
                "device": self.source,
                "revision": self.revision,
                "has_image": fresh,
            }

    def _capture(self) -> None:
        cap = None
        try:
            cap = open_color_camera(self.source, 640, 480, self.preview_fps)
            last_preview = 0.0
            failures = 0
            window_start = time.monotonic()
            counted = 0
            while not self.stopping.is_set():
                # The tested camera reports ~120fps even when lower FPS is requested.
                # Always grab incoming frames to avoid delivering a growing backlog.
                if not cap.grab():
                    failures += 1
                    if failures >= 30:
                        raise RuntimeError("USBカメラからフレームを取得できません")
                    self.stopping.wait(0.02)
                    continue
                now = time.monotonic()
                if now - last_preview < 1.0 / self.preview_fps:
                    continue
                ok, image = cap.retrieve()
                if not ok or image is None or not image.size:
                    failures += 1
                    if failures >= 30:
                        raise RuntimeError("USBカメラのフレームを復号できません")
                    continue
                failures = 0
                last_preview = now
                with self.lock:
                    colors = list(self.color_detections)
                    letters = list(self.letter_detections)
                    stable_colors = set(self._stable_colors())
                    stable_letters = set(self._stable_letters())
                    self.raw_frame = image
                    self.frame_stamp = now
                marked = image.copy()
                for d in colors:
                    x, y, w, h = d["bbox"]
                    c = d["color"]
                    cv2.rectangle(marked, (x, y), (x + w, y + h), COLOR_BGR[c], 2)
                    cv2.putText(marked, c.upper() + (" OK" if c in stable_colors else ""),
                                (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, COLOR_BGR[c], 2)
                for d in letters:
                    x, y, w, h = d["bbox"]
                    ch = d["letter"]
                    cv2.rectangle(marked, (x, y), (x + w, y + h), (255, 220, 80), 2)
                    cv2.putText(marked, ch + (" OK" if ch in stable_letters else "?"),
                                (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.65, (255, 220, 80), 2)
                success, jpeg = cv2.imencode(".jpg", marked, [cv2.IMWRITE_JPEG_QUALITY, 65])
                if success:
                    with self.lock:
                        self.frame = jpeg.tobytes()
                        self.message = "USBカメラ ライブ"
                        self.revision += 1
                counted += 1
                if now - window_start >= 1.0:
                    with self.lock:
                        self.captured_fps = counted / (now - window_start)
                    counted, window_start = 0, now
        except (ValueError, RuntimeError, OSError, cv2.error) as exc:
            with self.lock:
                self.message = f"カメラエラー: {exc}"
                self.frame = None
                self.raw_frame = None
            print(f"[CAMERA] {exc}", flush=True)
        finally:
            if cap is not None:
                cap.release()
            self.stopping.set()

    def _analyze_colors(self) -> None:
        while not self.stopping.wait(COLOR_INTERVAL_SECONDS):
            image = self.get_raw_frame()
            if image is None:
                with self.lock:
                    self.history.clear()
                    self.color_detections = []
                continue
            try:
                results = analyze_color_frame(image, roi=self.roi)
            except (cv2.error, ValueError) as exc:
                print(f"[CAMERA COLOR] {exc}", flush=True)
                continue
            with self.lock:
                self.history.append((time.monotonic(), {d["color"] for d in results}))
                self.color_detections = results

    def _analyze_letters(self) -> None:
        while not self.stopping.wait(OCR_INTERVAL_SECONDS):
            image = self.get_raw_frame()
            if image is None:
                with self.lock:
                    self.letter_history.clear()
                    self.letter_detections = []
                continue
            try:
                results = analyze_hsu_frame(image, self.roi)
            except Exception as exc:  # optional OCR can fail or time out
                with self.lock:
                    self.ocr_message = f"OCRエラー: {str(exc)[:120]}"
                    self.letter_history.clear()
                    self.letter_detections = []
                continue
            with self.lock:
                self.ocr_message = "H/S/U候補の確認中（試験機能）"
                self.letter_history.append((time.monotonic(), {d["letter"] for d in results}))
                self.letter_detections = results

    def new_video_track(self, fps: int = 30):
        """Build an aiortc video track fed by shared capture, never reopen V4L2."""
        from aiortc import VideoStreamTrack
        from av import VideoFrame

        camera = self
        target_fps = max(1, min(int(fps), 60))

        class SharedCameraTrack(VideoStreamTrack):
            def __init__(self):
                super().__init__()
                self.started_at = time.monotonic()
                self.frame_number = 0

            async def recv(self):
                deadline = self.started_at + self.frame_number / target_fps
                delay = deadline - time.monotonic()
                if delay > 0:
                    await asyncio.sleep(delay)
                img = camera.get_raw_frame()
                if img is None:
                    img = np.zeros((480, 640, 3), dtype=np.uint8)
                output = VideoFrame.from_ndarray(img, format="bgr24")
                output.pts = round(self.frame_number * 90000 / target_fps)
                output.time_base = Fraction(1, 90000)
                self.frame_number += 1
                return output

        return SharedCameraTrack()
