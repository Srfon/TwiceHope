import serial
import threading
import queue
import numpy as np
import struct
import cv2
import time
import math
from collections import deque

import board
import busio

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
)

# =========================================================
# CONFIG
# =========================================================

# ---------- LiDAR ----------
LIDAR_PORT = "/dev/ttyAMA2"
LIDAR_BAUD = 230400

FRAME_SIZE = 47
HEADER = 0x54

# ---------- Temporal Cloud ----------
POINT_LIFETIME = 0.25          # sec
MAX_TEMPORAL_POINTS = 4000

# ---------- Compensation ----------
YAW_ALPHA = 0.35               # 0.0~1.0
GYRO_PREDICT_DT = 0.015        # sec

# ---------- Sparse Sampling ----------
SPARSE_N = 3                   # 1=no sparse

# ---------- Draw ----------

WIDTH = 800
HEIGHT = 800
CX = WIDTH // 2
CY = HEIGHT // 2

DRAW_RANGE_MM = 3000.0
PIX = WIDTH / (DRAW_RANGE_MM * 2)

# ---------- LiDAR ----------
POINT_DT = 1.0 / 4500.0

# =========================================================
# CRC TABLE
# =========================================================

CRC8_TABLE = [
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
    0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
]

# =========================================================
# QUEUES
# =========================================================

raw_queue = queue.Queue(maxsize=4)
frame_queue = queue.Queue(maxsize=4)

# =========================================================
# SHARED
# =========================================================

imu_lock = threading.Lock()
imu_buffer = deque(maxlen=512)

temporal_cloud_lock = threading.Lock()
temporal_cloud = []

frame_counter = 0

# =========================================================
# UTILS
# =========================================================

def safe_put(q, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except:
            pass
        q.put_nowait(item)

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def angle_diff(a, b):
    d = a - b

    if d > 180:
        d -= 360

    if d < -180:
        d += 360

    return d

# =========================================================
# QUATERNION
# =========================================================

def quaternion_to_euler(i, j, k, r):

    # BNO085 -> Robot frame

    x = j
    y = -i
    z = k
    w = r

    # roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)

    roll = math.atan2(
        sinr_cosp,
        cosr_cosp
    )

    # pitch
    sinp = 2 * (w * y - z * x)

    if abs(sinp) >= 1:
        pitch = math.copysign(
            math.pi / 2,
            sinp
        )
    else:
        pitch = math.asin(sinp)

    # yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)

    yaw = math.atan2(
        siny_cosp,
        cosy_cosp
    )

    return (
        math.degrees(-roll),
        math.degrees(pitch),
        math.degrees(-yaw)
    )

# =========================================================
# IMU THREAD
# =========================================================

def imu_thread():

    i2c = busio.I2C(
        board.SCL,
        board.SDA,
        frequency=400000
    )

    bno = BNO08X_I2C(i2c)

    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    while True:

        try:

            qi, qj, qk, qr = bno.quaternion

            roll, pitch, yaw = quaternion_to_euler(
                qi, qj, qk, qr
            )

            gx, gy, gz = bno.gyro

            yaw = normalize_angle(yaw)

            ts = time.monotonic()

            with imu_lock:

                imu_buffer.append(
                    (
                        ts,
                        yaw,
                        math.degrees(gz)
                    )
                )

        except:
            pass

        time.sleep(0.005)

# =========================================================
# IMU INTERPOLATION
# =========================================================

def get_interpolated_yaw(target_time):

    with imu_lock:

        if len(imu_buffer) < 2:
            return None

        data = list(imu_buffer)

    for i in range(len(data) - 1):

        t0, yaw0, gz0 = data[i]
        t1, yaw1, gz1 = data[i + 1]

        if t0 <= target_time <= t1:

            ratio = (
                (target_time - t0) /
                (t1 - t0 + 1e-9)
            )

            dyaw = angle_diff(yaw1, yaw0)

            interp_yaw = yaw0 + dyaw * ratio

            interp_gz = gz0 + (gz1 - gz0) * ratio

            pred_yaw = (
                interp_yaw +
                interp_gz * GYRO_PREDICT_DT
            )

            pred_yaw = normalize_angle(pred_yaw)

            return pred_yaw

    return data[-1][1]

# =========================================================
# LIDAR RX THREAD
# =========================================================

def lidar_reader_thread():

    ser = serial.Serial(
        LIDAR_PORT,
        LIDAR_BAUD,
        timeout=0.01
    )

    ser.write(
        b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2"
    )

    while True:

        data = ser.read(4096)

        if data:
            safe_put(raw_queue, data)

# =========================================================
# PARSER THREAD
# =========================================================

def parser_thread():

    global frame_counter

    buf = bytearray()

    while True:

        data = raw_queue.get()

        while True:
            try:
                data = raw_queue.get_nowait()
            except queue.Empty:
                break

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

            crc = 0

            for x in frame[:-1]:
                crc = CRC8_TABLE[crc ^ x]

            if crc == frame[-1]:

                ts = time.monotonic()

                safe_put(
                    frame_queue,
                    (
                        ts,
                        frame_counter,
                        frame
                    )
                )

                frame_counter += 1

                buf = buf[idx + FRAME_SIZE:]

            else:
                buf = buf[idx + 1:]

# =========================================================
# POINT PROCESS THREAD
# =========================================================

def point_process_thread():

    global temporal_cloud

    while True:

        frame_time, frame_id, frame = frame_queue.get()

        start_angle = (
            struct.unpack_from("<H", frame, 4)[0]
            / 100.0
        )

        end_angle = (
            struct.unpack_from("<H", frame, 42)[0]
            / 100.0
        )

        if end_angle < start_angle:
            end_angle += 360.0

        raw = frame[6:6 + 12 * 3]

        raw = np.frombuffer(
            raw,
            dtype=np.uint8
        ).reshape(-1, 3)

        dists = (
            raw[:, 0].astype(np.uint16)
            |
            (raw[:, 1].astype(np.uint16) << 8)
        )

        valid = dists > 0

        dists = dists[valid]

        idxs = np.arange(12)[valid]

        angles = (
            start_angle
            +
            (end_angle - start_angle)
            * idxs / 11.0
        )

        now = time.monotonic()

        new_points = []

        offset = frame_id % SPARSE_N

        for i, (dist, angle_deg) in enumerate(zip(dists, angles)):

            if ((i + offset) % SPARSE_N) != 0:
                continue

            point_time = (
                frame_time
                +
                i * POINT_DT
            )

            imu_yaw = get_interpolated_yaw(
                point_time
            )

            if imu_yaw is None:
                continue

            corrected_angle = (
                angle_deg
                -
                imu_yaw * YAW_ALPHA
            )

            corrected_angle = normalize_angle(
                corrected_angle
            )

            rad = np.deg2rad(corrected_angle)

            x = dist * np.sin(rad)
            y = dist * np.cos(rad)

            new_points.append(
                (
                    x,
                    y,
                    point_time
                )
            )

        with temporal_cloud_lock:

            temporal_cloud.extend(new_points)

            temporal_cloud = [
                p for p in temporal_cloud
                if (now - p[2]) < POINT_LIFETIME
            ]

            if len(temporal_cloud) > MAX_TEMPORAL_POINTS:
                temporal_cloud = temporal_cloud[-MAX_TEMPORAL_POINTS:]

# =========================================================
# DRAW THREAD
# =========================================================

def draw_thread():

    cv2.namedWindow(
        "Temporal LiDAR",
        cv2.WINDOW_NORMAL
    )

    last_time = time.monotonic()
    fps = 0

    while True:

        now = time.monotonic()

        fps = 1.0 / (
            now - last_time + 1e-9
        )

        last_time = now

        img = np.zeros(
            (HEIGHT, WIDTH, 3),
            dtype=np.uint8
        )

        with temporal_cloud_lock:

            cloud = list(temporal_cloud)

        for x, y, ts in cloud:

            age = now - ts

            weight = (
                1.0
                -
                age / POINT_LIFETIME
            )

            if weight < 0:
                continue

            brightness = max(
                0,
                min(
                    255,
                    int(weight * 255)
                )
            )

            px = int(CX + x * PIX)
            py = int(CY - y * PIX)

            if (
                0 <= px < WIDTH
                and
                0 <= py < HEIGHT
            ):
                img[py, px] = (
                    0,
                    brightness,
                    0
                )

        # center
        cv2.circle(
            img,
            (CX, CY),
            4,
            (0, 0, 255),
            -1
        )

        # forward axis
        cv2.line(
            img,
            (CX, CY),
            (CX, CY - 80),
            (255, 0, 0),
            2
        )

        cv2.putText(
            img,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255,255,255),
            2
        )

        cv2.putText(
            img,
            f"Points: {len(cloud)}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255,255,255),
            2
        )

        cv2.putText(
            img,
            f"YawAlpha: {YAW_ALPHA:.2f}",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255,255,255),
            2
        )

        cv2.imshow(
            "Temporal LiDAR",
            img
        )

        key = cv2.waitKey(1)

        if key == 27:
            break

# =========================================================
# MAIN
# =========================================================

threading.Thread(
    target=imu_thread,
    daemon=True
).start()

threading.Thread(
    target=lidar_reader_thread,
    daemon=True
).start()

threading.Thread(
    target=parser_thread,
    daemon=True
).start()

threading.Thread(
    target=point_process_thread,
    daemon=True
).start()

draw_thread()