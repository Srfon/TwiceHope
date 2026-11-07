"""
lidar_imu_st7789.py
-------------------------------------------------
ORBBEC MS200 + BNO085
Temporal LiDAR Viewer for ST7789 240x240 LCD

- temporal point cloud
- IMU delta yaw correction
- future timestamp correction
- ST7789 LCD output
- internal high resolution render -> downscale

Raspberry Pi 5
Python 3.11+
"""

import serial
import threading
import queue
import numpy as np
import struct
import cv2
import time
import math
import logging
from collections import deque

import board
import busio

from PIL import Image

import ST7789

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE
)

# ===========================================================
# CONFIG
# ===========================================================

# ---------- LiDAR ----------
LIDAR_PORT = "/dev/ttyAMA0"
LIDAR_BAUD = 230400

FRAME_SIZE = 47
HEADER = 0x54
POINTS_PER_FRAME = 12
POINT_RATE_HZ = 4500

# ---------- Temporal ----------
POINT_LIFETIME = 0.6
MAX_TEMPORAL_POINTS = 8000

# ---------- IMU ----------
YAW_ALPHA = 1.2
GYRO_PREDICT_DT = 0.015
DELTA_YAW_WINDOW = 0.08

# ---------- Sparse ----------
SPARSE_N = 2

# ---------- Render ----------
RENDER_W = 480
RENDER_H = 480

LCD_W = 240
LCD_H = 240

CX = RENDER_W // 2
CY = RENDER_H // 2

DRAW_RANGE_MM = 500.0

PIX = RENDER_W / (DRAW_RANGE_MM * 2)

DRAW_FPS_LIMIT = 20

POINT_RADIUS = 1

SHOW_OSD = True

# ===========================================================
# LOG
# ===========================================================

logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s [%(threadName)s] %(levelname)s: %(message)s"
)

log = logging.getLogger("lidar")

# ===========================================================
# ST7789
# ===========================================================

disp = ST7789.ST7789(
    port=0,
    cs=0,
    dc=25,
    rst=24,
    width=240,
    height=240,
    rotation=0,
    spi_speed_hz=20_000_000
)

disp.begin()

disp._spi.mode = 3

disp.reset()
disp._init()

# ===========================================================
# CRC8
# ===========================================================

CRC8_TABLE = bytes([
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,
    0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,
    0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,
    0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,
    0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,
    0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,
    0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,
    0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,
    0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,
    0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,
    0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,
    0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,
    0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,
    0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,
    0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,
    0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,
    0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8,
])

# ===========================================================
# UTILS
# ===========================================================

def crc8(frame):
    crc = 0
    for b in frame:
        crc = CRC8_TABLE[crc ^ b]
    return crc

def normalize_angle(a):

    while a > 180:
        a -= 360

    while a < -180:
        a += 360

    return a

def angle_diff(a, b):

    d = a - b

    if d > 180:
        d -= 360

    if d < -180:
        d += 360

    return d

def safe_put(q, item):

    if q.full():

        try:
            q.get_nowait()

        except queue.Empty:
            pass

    try:
        q.put_nowait(item)

    except queue.Full:
        pass

# ===========================================================
# QUEUE
# ===========================================================

raw_queue = queue.Queue(maxsize=8)
frame_queue = queue.Queue(maxsize=16)

# ===========================================================
# SHARED
# ===========================================================

imu_lock = threading.Lock()
imu_buffer = deque(maxlen=2048)

imu_zero_yaw = None

cloud_lock = threading.Lock()
temporal_cloud = []

# ===========================================================
# STATS
# ===========================================================

stats_lock = threading.Lock()

stats = {
    "imu_hz": 0.0,
    "cloud_n": 0,
    "yaw": 0.0,
    "imu_miss": 0,
}

# ===========================================================
# QUATERNION
# ===========================================================

def quat_to_euler(qi, qj, qk, qr):

    x, y, z, w = qj, -qi, qk, qr

    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2 * (x*x + y*y)

    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w*y - z*x)

    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi/2, sinp)

    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)

    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (
        math.degrees(-roll),
        math.degrees(pitch),
        math.degrees(-yaw)
    )

# ===========================================================
# IMU THREAD
# ===========================================================

def imu_thread():

    global imu_zero_yaw

    i2c = busio.I2C(
        board.SCL,
        board.SDA,
        frequency=400_000
    )

    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    prev_ts = time.monotonic()

    while True:

        try:

            qi, qj, qk, qr = bno.quaternion

            _, _, yaw = quat_to_euler(
                qi,
                qj,
                qk,
                qr
            )

            gx, gy, gz = bno.gyro

            yaw = normalize_angle(yaw)

            if imu_zero_yaw is None:
                imu_zero_yaw = yaw

            yaw = normalize_angle(yaw - imu_zero_yaw)

            ts = time.monotonic()

            gz_deg = math.degrees(gz)

            with imu_lock:
                imu_buffer.append(
                    (ts, yaw, gz_deg)
                )

            dt = ts - prev_ts
            prev_ts = ts

            if dt > 0:

                with stats_lock:
                    stats["imu_hz"] = 1.0 / dt
                    stats["yaw"] = yaw

        except Exception as e:
            log.warning("IMU error: %s", e)

        time.sleep(0.004)

# ===========================================================
# INTERPOLATION
# ===========================================================

def get_interpolated_yaw(target_time):

    with imu_lock:

        if len(imu_buffer) < 2:
            return None

        data = list(imu_buffer)

    lo = 0
    hi = len(data) - 1

    while lo < hi - 1:

        mid = (lo + hi) // 2

        if data[mid][0] <= target_time:
            lo = mid
        else:
            hi = mid

    t0, y0, g0 = data[lo]
    t1, y1, g1 = data[hi]

    ratio = (target_time - t0) / (t1 - t0 + 1e-9)

    dyaw = angle_diff(y1, y0)

    interp_yaw = y0 + dyaw * ratio
    interp_gz = g0 + (g1 - g0) * ratio

    pred_yaw = normalize_angle(
        interp_yaw + interp_gz * GYRO_PREDICT_DT
    )

    return pred_yaw

# ===========================================================
# DELTA YAW
# ===========================================================

def get_delta_yaw(target_time):

    yaw_now = get_interpolated_yaw(target_time)

    yaw_prev = get_interpolated_yaw(
        target_time - DELTA_YAW_WINDOW
    )

    if yaw_now is None or yaw_prev is None:
        return None

    return angle_diff(yaw_now, yaw_prev)

# ===========================================================
# LIDAR RX
# ===========================================================

def lidar_reader_thread():

    ser = serial.Serial(
        LIDAR_PORT,
        LIDAR_BAUD,
        timeout=0.01
    )

    ser.write(b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2")

    while True:

        try:

            data = ser.read(4096)

            if data:
                safe_put(
                    raw_queue,
                    (time.monotonic(), data)
                )

        except Exception as e:
            log.warning("Serial error: %s", e)

            time.sleep(0.1)

# ===========================================================
# PARSER
# ===========================================================

def parser_thread():

    buf = bytearray()

    while True:

        try:
            recv_ts, data = raw_queue.get(timeout=0.5)

        except queue.Empty:
            continue

        buf.extend(data)

        while True:

            idx = buf.find(bytes([HEADER]))

            if idx < 0:
                buf.clear()
                break

            if len(buf) - idx < FRAME_SIZE:
                buf = buf[idx:]
                break

            frame = buf[idx:idx + FRAME_SIZE]

            if crc8(frame[:-1]) == frame[-1]:

                safe_put(
                    frame_queue,
                    (time.monotonic(), bytes(frame))
                )

                buf = buf[idx + FRAME_SIZE:]

            else:
                buf = buf[idx + 1:]

# ===========================================================
# POINT PROCESS
# ===========================================================

def point_process_thread():

    global temporal_cloud

    POINT_DT = 1.0 / POINT_RATE_HZ

    imu_miss = 0

    while True:

        try:
            frame_time, frame = frame_queue.get(timeout=0.5)

        except queue.Empty:
            continue

        start_angle = (
            struct.unpack_from("<H", frame, 4)[0] / 100.0
        )

        end_angle = (
            struct.unpack_from("<H", frame, 42)[0] / 100.0
        )

        if end_angle < start_angle:
            end_angle += 360.0

        raw = np.frombuffer(
            frame[6:6 + POINTS_PER_FRAME * 3],
            dtype=np.uint8
        )

        raw = raw.reshape(POINTS_PER_FRAME, 3)

        dists = (
            raw[:, 0].astype(np.uint16)
            |
            (raw[:, 1].astype(np.uint16) << 8)
        )

        valid = np.where(dists > 0)[0]

        if len(valid) == 0:
            continue

        angles = (
            start_angle
            +
            (end_angle - start_angle)
            *
            valid
            /
            (POINTS_PER_FRAME - 1)
        )

        new_points = []

        now = time.monotonic()

        for pi, angle_deg in zip(valid, angles):

            if (pi % SPARSE_N) != 0:
                continue

            point_time = frame_time + pi * POINT_DT

            point_time = min(
                point_time,
                time.monotonic()
            )

            delta_yaw = get_delta_yaw(point_time)

            if delta_yaw is None:

                delta_yaw = 0.0
                imu_miss += 1

            corrected = normalize_angle(
                angle_deg
                -
                delta_yaw * YAW_ALPHA
            )

            rad = math.radians(corrected)

            dist = int(dists[pi])

            x = dist * math.sin(rad)
            y = dist * math.cos(rad)

            new_points.append(
                (x, y, point_time)
            )

        with cloud_lock:

            temporal_cloud.extend(new_points)

            cutoff = now - POINT_LIFETIME

            temporal_cloud = [
                p for p in temporal_cloud
                if p[2] >= cutoff
            ]

            if len(temporal_cloud) > MAX_TEMPORAL_POINTS:
                temporal_cloud = temporal_cloud[-MAX_TEMPORAL_POINTS:]

            cloud_n = len(temporal_cloud)

        with stats_lock:
            stats["cloud_n"] = cloud_n
            stats["imu_miss"] = imu_miss

# ===========================================================
# DRAW
# ===========================================================

def draw_thread():

    prev = time.monotonic()

    while True:

        now = time.monotonic()

        dt = now - prev

        if dt < (1.0 / DRAW_FPS_LIMIT):

            time.sleep(0.001)
            continue

        prev = now

        with cloud_lock:
            cloud = list(temporal_cloud)

        img = np.zeros(
            (RENDER_H, RENDER_W, 3),
            dtype=np.uint8
        )

        if cloud:

            arr = np.array(cloud, dtype=np.float32)

            xs = arr[:, 0]
            ys = arr[:, 1]
            ts = arr[:, 2]

            ages = now - ts

            brightness = np.clip(
                1.0 - ages / POINT_LIFETIME,
                0,
                1
            )

            px = (CX + xs * PIX).astype(np.int32)
            py = (CY - ys * PIX).astype(np.int32)

            mask = (
                (px >= 0)
                &
                (px < RENDER_W)
                &
                (py >= 0)
                &
                (py < RENDER_H)
            )

            px = px[mask]
            py = py[mask]

            bv = (
                brightness[mask] * 255
            ).astype(np.uint8)

            order = np.argsort(bv)

            for i in order:

                cv2.circle(
                    img,
                    (int(px[i]), int(py[i])),
                    POINT_RADIUS,
                    (0, int(bv[i]), 0),
                    -1,
                    cv2.LINE_AA
                )

        cv2.circle(
            img,
            (CX, CY),
            3,
            (0, 0, 255),
            -1
        )

        cv2.line(
            img,
            (CX, CY),
            (CX, CY - 60),
            (255, 80, 0),
            2
        )

        if SHOW_OSD:

            with stats_lock:
                s = dict(stats)

            lines = [
                f"Yaw {s['yaw']:+.1f}",
                f"P {s['cloud_n']}",
                f"IMU {s['imu_hz']:.0f}Hz",
            ]

            for i, line in enumerate(lines):

                cv2.putText(
                    img,
                    line,
                    (8, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA
                )

        # ===================================================
        # Downscale -> LCD
        # ===================================================

        small = cv2.resize(
            img,
            (LCD_W, LCD_H),
            interpolation=cv2.INTER_AREA
        )

        small = cv2.cvtColor(
            small,
            cv2.COLOR_BGR2RGB
        )

        pil_img = Image.fromarray(small)

        disp.display(pil_img)

# ===========================================================
# MAIN
# ===========================================================

if __name__ == "__main__":

    threads = [

        threading.Thread(
            target=imu_thread,
            daemon=True
        ),

        threading.Thread(
            target=lidar_reader_thread,
            daemon=True
        ),

        threading.Thread(
            target=parser_thread,
            daemon=True
        ),

        threading.Thread(
            target=point_process_thread,
            daemon=True
        ),
    ]

    for t in threads:
        t.start()

    draw_thread()