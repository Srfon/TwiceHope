from __future__ import annotations

import cv2
import numpy as np

from .protocol import Measurement


class ScanRenderer:
    def __init__(self, size_px: int, range_mm: int, jpeg_quality: int = 80) -> None:
        self.size_px = size_px
        self.range_mm = range_mm
        self.jpeg_quality = jpeg_quality
        self.center = size_px // 2
        self.scale = size_px / (2.0 * range_mm)
        self._background = self._make_background()

    def _make_background(self) -> np.ndarray:
        image = np.full((self.size_px, self.size_px, 3), (13, 19, 25), np.uint8)
        step = 500 if self.range_mm >= 2000 else 100
        for distance in range(step, self.range_mm + 1, step):
            radius = round(distance * self.scale)
            color = (43, 57, 69) if distance < self.range_mm else (70, 88, 102)
            cv2.circle(image, (self.center, self.center), radius, color, 1, cv2.LINE_AA)
        cv2.line(image, (self.center, 0), (self.center, self.size_px - 1), (43, 57, 69), 1)
        cv2.line(image, (0, self.center), (self.size_px - 1, self.center), (43, 57, 69), 1)
        cv2.arrowedLine(image, (self.center, self.center), (self.center, 24), (0, 190, 255), 2, tipLength=0.04)
        cv2.putText(image, "FRONT", (self.center + 8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 190, 255), 1, cv2.LINE_AA)
        cv2.circle(image, (self.center, self.center), 4, (40, 70, 255), -1, cv2.LINE_AA)
        return image

    def render(self, points: list[Measurement] | tuple[Measurement, ...]) -> np.ndarray:
        image = self._background.copy()
        if not points:
            return image
        angles = np.deg2rad(np.fromiter((p.angle_deg for p in points), dtype=np.float32))
        distances = np.fromiter((p.distance_mm for p in points), dtype=np.float32)
        xs = distances * np.sin(angles)
        ys = distances * np.cos(angles)
        px = np.rint(self.center + xs * self.scale).astype(np.int32)
        py = np.rint(self.center - ys * self.scale).astype(np.int32)
        mask = (px >= 0) & (px < self.size_px) & (py >= 0) & (py < self.size_px)
        image[py[mask], px[mask]] = (80, 240, 120)
        return image

    def jpeg(self, image: np.ndarray) -> bytes:
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(self.jpeg_quality)]
        )
        if not ok:
            raise RuntimeError("OpenCV JPEG encoding failed")
        return encoded.tobytes()
