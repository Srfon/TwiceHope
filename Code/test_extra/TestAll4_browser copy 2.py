#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import copy
import signal
import struct
import threading
import queue
import select
import io
from collections import deque

try:
    import tty
    import termios
    TTY_LIB_OK = True
except ImportError:
    # Windows等、termios非対応環境向けのフォールバック
    TTY_LIB_OK = False

from PIL import Image, ImageDraw, ImageFont

from browser_viewer import viewer  # ローカルの viewer.py (ブラウザ表示用)

# ==================================================================
# CONFIG : 有効/無効フラグと各種パラメータ
#          (配線されていないセンサーは False にすれば安全にスキップされる)
# ==================================================================

ENABLE_IMU = True             # BNO085 (IMU)
ENABLE_TCA_COLOR = True        # TCA9548A CH0/CH1 : TCS34725 カラーセンサー
ENABLE_TCA_DISTANCE = True     # TCA9548A CH2/CH3 : VL53L1X 距離センサー
ENABLE_LIDAR = True            # ORBBEC MS200 LiDAR
ENABLE_BUTTON = True           # GPIOロッカースイッチ(右/左)での手動ページ切り替え
ENABLE_KEYBOARD = True         # ラズパイ側/SSH端末のキーボード ← → キーでページ切り替え
ENABLE_CONTROLLER = True       # evdev対応ゲームコントローラでの操作 (スティック/ボタン)

AUTO_ROTATE = False             # ページを自動で巡回させるか (手動操作後は一定時間停止する)
PAGE_DURATION_SEC = 4.0        # 自動巡回時の1ページあたりの表示秒数
MANUAL_HOLD_SEC = 6.0          # 手動でページ切替した後、自動巡回を再開するまでの待機秒数
DISPLAY_FPS = 15               # 画面更新のフレームレート上限

# ---------- I2C アドレス ----------
TCA_ADDR = 0x70
TCS_ADDR = 0x29     # TCS34725 (RGBカラーセンサー)
VL_ADDR = 0x29       # VL53L1X  (別チャンネルなのでアドレス重複OK)

# ---------- GPIOロッカースイッチ (右/左 2方向) ----------
RIGHT_PIN = 17
LEFT_PIN = 27
# このスイッチは構造上、右へ倒す途中で一瞬左が反応することがあるため、
# 「反対側が押されてからSWITCH_CONFIRM_SEC秒以上経過後に目的方向が
#  押された場合のみ」を有効な操作として確定させる(誤動作防止)
SWITCH_CONFIRM_SEC = 0.3

# ---------- 描画キャンバス / viewer.py (ブラウザ表示) ----------
CANVAS_W = 240
CANVAS_H = 240
JPEG_QUALITY = 85               # ブラウザへ送るJPEGの画質(1-95目安)
VIEWER_HOST = "0.0.0.0"
VIEWER_PORT = 8000
VIEWER_REFRESH_MS = 200
VIEWER_IMAGE_NAME = "dashboard"

# ---------- LiDAR (ORBBEC MS200) ----------
LIDAR_PORT = "/dev/ttyAMA0"
LIDAR_BAUD = 230400
LIDAR_FRAME_SIZE = 47
LIDAR_HEADER = 0x54
LIDAR_POINTS_PER_FRAME = 12
LIDAR_POINT_RATE_HZ = 4500

# ---------- LiDAR 時系列点群 ----------
POINT_LIFETIME = 0.6            # 点の寿命(秒)
MAX_TEMPORAL_POINTS = 4000
YAW_ALPHA = 1.2
GYRO_PREDICT_DT = 0.015
DELTA_YAW_WINDOW = 0.08
SPARSE_N = 2                     # 間引き係数
DRAW_RANGE_MM = 1500.0           # 画面に表示する最大距離(mm) : コントローラでズーム調整される初期値
LIDAR_RANGE_MIN_MM = 300.0       # ズームの最小距離(最大ズームイン)
LIDAR_RANGE_MAX_MM = 4000.0      # ズームの最大距離(最大ズームアウト)
LIDAR_RANGE_STEP_PER_SEC = 1800.0  # スティックを全開に倒した時の変化速度(mm/秒)

# ---------- ゲームコントローラ (evdev) ----------
CONTROLLER_DEVICE = None          # 例: "/dev/input/event3" (Noneなら自動検出を試みる)
CONTROLLER_DEADZONE = 6000         # スティックの遊び(不感帯)。絶対値がこれ未満は無視
CONTROLLER_AXIS_MAX = 32767        # 一般的なコントローラの軸最大値 (機種によって異なる場合あり)
STICK_PAGE_THRESHOLD = 22000       # 左スティックでページ送りする際のしきい値(倒しきり判定)
LIDAR_MAX_DRAW_POINTS = 1500     # 1フレームで描画する点の上限(処理負荷対策)

# ==================================================================
# 任意ライブラリの読み込み (無ければ該当機能だけを自動的に無効化する)
# ==================================================================

try:
    import board
    import busio
    HW_I2C_LIB_OK = True
except ImportError:
    HW_I2C_LIB_OK = False

try:
    from smbus2 import SMBus
    SMBUS_LIB_OK = True
except ImportError:
    SMBUS_LIB_OK = False

try:
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_bno08x import (
        BNO_REPORT_ROTATION_VECTOR,
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_GYROSCOPE,
    )
    BNO_LIB_OK = True
except ImportError:
    BNO_LIB_OK = False

try:
    import adafruit_vl53l1x
    VL53_LIB_OK = True
except ImportError:
    VL53_LIB_OK = False

try:
    import serial
    SERIAL_LIB_OK = True
except ImportError:
    SERIAL_LIB_OK = False

try:
    import numpy as np
    NUMPY_LIB_OK = True
except ImportError:
    NUMPY_LIB_OK = False

try:
    from gpiozero import Button
    GPIOZERO_LIB_OK = True
except ImportError:
    GPIOZERO_LIB_OK = False

try:
    from evdev import InputDevice, ecodes, list_devices
    EVDEV_LIB_OK = True
except ImportError:
    EVDEV_LIB_OK = False

# 実際に使う機能は「フラグON」かつ「ライブラリが揃っている」場合のみ
USE_IMU = ENABLE_IMU and BNO_LIB_OK and HW_I2C_LIB_OK
USE_TCA_COLOR = ENABLE_TCA_COLOR and SMBUS_LIB_OK
USE_TCA_DISTANCE = ENABLE_TCA_DISTANCE and VL53_LIB_OK and HW_I2C_LIB_OK and SMBUS_LIB_OK
USE_LIDAR = ENABLE_LIDAR and SERIAL_LIB_OK and NUMPY_LIB_OK
USE_BUTTON = ENABLE_BUTTON and GPIOZERO_LIB_OK
USE_KEYBOARD = ENABLE_KEYBOARD and TTY_LIB_OK
USE_CONTROLLER = ENABLE_CONTROLLER and EVDEV_LIB_OK


# ==================================================================
# 共有状態 (各スレッド -> 描画スレッド)
# ==================================================================

state_lock = threading.Lock()

sensor_state = {
    "imu": {
        "ok": False, "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "ax": 0.0, "ay": 0.0, "az": 0.0, "hz": 0.0,
    },
    "color0": {"ok": False, "c": 0, "r": 0, "g": 0, "b": 0},
    "color1": {"ok": False, "c": 0, "r": 0, "g": 0, "b": 0},
    "dist2": {"ok": False, "mm": None},
    "dist3": {"ok": False, "mm": None},
    "lidar": {"ok": False, "points": 0, "hz": 0.0, "miss": 0},
}

# I2Cバスを複数スレッドで共有するための排他ロック
i2c_lock = threading.Lock()

# アプリ全体の実行フラグ (Ctrl+C で False にして全スレッドを止める)
running = True

# ページ切り替え制御
current_page_idx = 0
page_lock = threading.Lock()
last_manual_switch = 0.0   # 最後に手動でページを切り替えた時刻 (自動巡回の一時停止に使う)


def change_page(delta, num_pages):
    """ページをdelta分だけ進める/戻す (手動操作用の共通関数)"""
    global current_page_idx, last_manual_switch
    with page_lock:
        current_page_idx = (current_page_idx + delta) % num_pages
        last_manual_switch = time.monotonic()


def set_page_index(idx, num_pages):
    """ページ番号を直接指定する (コントローラのボタンでの直接ジャンプ用)"""
    global current_page_idx, last_manual_switch
    with page_lock:
        current_page_idx = idx % num_pages
        last_manual_switch = time.monotonic()


# 実行中に変更可能な設定値 (LiDARズーム倍率・自動巡回ON/OFFなど)
settings_lock = threading.Lock()
runtime_settings = {
    "draw_range_mm": DRAW_RANGE_MM,   # LiDARページの表示レンジ(ズーム)
    "auto_rotate": AUTO_ROTATE,       # 自動ページ巡回 ON/OFF
}


def adjust_lidar_range(delta_mm):
    """LiDAR描画距離(ズーム)を相対的に変更する。範囲外にはクランプする。"""
    with settings_lock:
        new_range = runtime_settings["draw_range_mm"] + delta_mm
        new_range = max(LIDAR_RANGE_MIN_MM, min(LIDAR_RANGE_MAX_MM, new_range))
        runtime_settings["draw_range_mm"] = new_range


def reset_lidar_range():
    with settings_lock:
        runtime_settings["draw_range_mm"] = DRAW_RANGE_MM
    print("[PAD] LiDAR描画距離をリセットしました")


def toggle_auto_rotate():
    with settings_lock:
        runtime_settings["auto_rotate"] = not runtime_settings["auto_rotate"]
        state = "ON" if runtime_settings["auto_rotate"] else "OFF"
    print(f"[PAD] 自動ページ巡回: {state}")

# LiDAR用: IMUの角度履歴 (時刻合わせ用)
imu_buffer = deque(maxlen=2048)
imu_buffer_lock = threading.Lock()
imu_zero_yaw = None

# LiDAR用: 時系列点群
cloud_lock = threading.Lock()
temporal_cloud = []


def set_state(key, **kwargs):
    """sensor_state の該当キーをロックしながら更新する共通ヘルパー"""
    with state_lock:
        sensor_state[key].update(kwargs)


def get_state_snapshot():
    """描画用にsensor_stateのコピーを取得する"""
    with state_lock:
        return copy.deepcopy(sensor_state)


# ==================================================================
# 角度計算まわりのユーティリティ
# ==================================================================

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


def quat_to_euler(qi, qj, qk, qr):
    """
    BNO085の座標系(X=右,Y=前,Z=上)を一般的な機体座標系に変換してから
    クォータニオンをオイラー角(deg)に変換する
    """
    x, y, z, w = qj, -qi, qk, qr

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(-roll), math.degrees(pitch), math.degrees(-yaw)


def safe_queue_put(q, item):
    """キューが満杯なら古いものを捨てて新しいものを入れる"""
    if q.full():
        try:
            q.get_nowait()
        except queue.Empty:
            pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass


# ==================================================================
# IMU (BNO085) スレッド
# ==================================================================

def imu_thread():
    global imu_zero_yaw

    if not USE_IMU:
        return

    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
        bno = BNO08X_I2C(i2c)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
    except Exception as e:
        print("[IMU] 初期化に失敗しました:", e)
        set_state("imu", ok=False)
        return

    prev_ts = time.monotonic()

    while running:
        try:
            with i2c_lock:
                qi, qj, qk, qr = bno.quaternion
                ax, ay, az = bno.linear_acceleration
                gx, gy, gz = bno.gyro

            roll, pitch, yaw = quat_to_euler(qi, qj, qk, qr)
            roll = normalize_angle(roll)
            pitch = normalize_angle(pitch)
            yaw = normalize_angle(yaw)

            # LiDAR用: ゼロ点補正した yaw を履歴バッファに積む
            if imu_zero_yaw is None:
                imu_zero_yaw = yaw
            yaw_for_lidar = normalize_angle(yaw - imu_zero_yaw)
            gz_deg = math.degrees(gz)

            now = time.monotonic()
            with imu_buffer_lock:
                imu_buffer.append((now, yaw_for_lidar, gz_deg))

            dt = now - prev_ts
            prev_ts = now
            hz = (1.0 / dt) if dt > 0 else 0.0

            set_state(
                "imu",
                ok=True,
                roll=roll, pitch=pitch, yaw=yaw,
                ax=ax, ay=ay, az=az,
                hz=hz,
            )

        except Exception as e:
            print("[IMU] 読み取りエラー:", e)
            set_state("imu", ok=False)
            time.sleep(0.5)

        time.sleep(0.02)


# ==================================================================
# TCA9548A + TCS34725 (カラーセンサー) スレッド
# ==================================================================

def tca_select_channel(bus, ch):
    bus.write_byte(TCA_ADDR, 1 << ch)


def tcs_init(bus):
    bus.write_byte_data(TCS_ADDR, 0x80, 0x03)
    time.sleep(0.01)


def tcs_read_rgbc(bus):
    c = bus.read_word_data(TCS_ADDR, 0x94)
    r = bus.read_word_data(TCS_ADDR, 0x96)
    g = bus.read_word_data(TCS_ADDR, 0x98)
    b = bus.read_word_data(TCS_ADDR, 0x9A)
    return c, r, g, b


def tca_color_thread():
    if not USE_TCA_COLOR:
        return

    try:
        bus = SMBus(1)
        with i2c_lock:
            tca_select_channel(bus, 0)
            tcs_init(bus)
            tca_select_channel(bus, 1)
            tcs_init(bus)
    except Exception as e:
        print("[COLOR] 初期化に失敗しました:", e)
        set_state("color0", ok=False)
        set_state("color1", ok=False)
        return

    while running:
        for ch, key in ((0, "color0"), (1, "color1")):
            try:
                with i2c_lock:
                    tca_select_channel(bus, ch)
                    c, r, g, b = tcs_read_rgbc(bus)
                set_state(key, ok=True, c=c, r=r, g=g, b=b)
            except Exception as e:
                set_state(key, ok=False)

        time.sleep(0.1)


# ==================================================================
# TCA9548A + VL53L1X (距離センサー) スレッド
# ==================================================================

def tca_distance_thread():
    if not USE_TCA_DISTANCE:
        return

    try:
        bus = SMBus(1)
        i2c = busio.I2C(board.SCL, board.SDA)

        with i2c_lock:
            tca_select_channel(bus, 2)
            vl_ch2 = adafruit_vl53l1x.VL53L1X(i2c)
            vl_ch2.start_ranging()

            tca_select_channel(bus, 3)
            vl_ch3 = adafruit_vl53l1x.VL53L1X(i2c)
            vl_ch3.start_ranging()

    except Exception as e:
        print("[DIST] 初期化に失敗しました:", e)
        set_state("dist2", ok=False)
        set_state("dist3", ok=False)
        return

    sensors = ((2, "dist2", vl_ch2), (3, "dist3", vl_ch3))

    while running:
        for ch, key, sensor in sensors:
            try:
                with i2c_lock:
                    tca_select_channel(bus, ch)
                    mm = sensor.distance
                if mm is not None:
                    mm = mm * 10.0  # cm -> mm (adafruit_vl53l1xはcm単位で返す)
                set_state(key, ok=True, mm=mm)
            except Exception:
                set_state(key, ok=False, mm=None)

        time.sleep(0.05)


# ==================================================================
# LiDAR (ORBBEC MS200) : CRC8テーブル
# ==================================================================

CRC8_TABLE = bytes([
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE,
    0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07,
    0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1,
    0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18,
    0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90,
    0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39,
    0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F,
    0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26,
    0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2,
    0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B,
    0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD,
    0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64,
    0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC,
    0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3,
    0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A,
    0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
])


def crc8(frame):
    crc = 0
    for b in frame:
        crc = CRC8_TABLE[crc ^ b]
    return crc


# LiDAR用の内部キュー
lidar_raw_queue = queue.Queue(maxsize=8)
lidar_frame_queue = queue.Queue(maxsize=16)


def get_interpolated_yaw(target_time):
    with imu_buffer_lock:
        if len(imu_buffer) < 2:
            return None
        data = list(imu_buffer)

    lo, hi = 0, len(data) - 1
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

    return normalize_angle(interp_yaw + interp_gz * GYRO_PREDICT_DT)


def get_delta_yaw(target_time):
    yaw_now = get_interpolated_yaw(target_time)
    yaw_prev = get_interpolated_yaw(target_time - DELTA_YAW_WINDOW)
    if yaw_now is None or yaw_prev is None:
        return None
    return angle_diff(yaw_now, yaw_prev)


def lidar_reader_thread():
    if not USE_LIDAR:
        return
    try:
        ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.01)
        ser.write(b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2")
    except Exception as e:
        print("[LIDAR] シリアルポートを開けませんでした:", e)
        set_state("lidar", ok=False)
        return

    while running:
        try:
            data = ser.read(4096)
            if data:
                safe_queue_put(lidar_raw_queue, (time.monotonic(), data))
        except Exception as e:
            print("[LIDAR] 受信エラー:", e)
            time.sleep(0.1)


def lidar_parser_thread():
    if not USE_LIDAR:
        return

    buf = bytearray()

    while running:
        try:
            recv_ts, data = lidar_raw_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        buf.extend(data)

        while True:
            idx = buf.find(bytes([LIDAR_HEADER]))
            if idx < 0:
                buf.clear()
                break
            if len(buf) - idx < LIDAR_FRAME_SIZE:
                buf = buf[idx:]
                break

            frame = buf[idx:idx + LIDAR_FRAME_SIZE]

            if crc8(frame[:-1]) == frame[-1]:
                safe_queue_put(lidar_frame_queue, (time.monotonic(), bytes(frame)))
                buf = buf[idx + LIDAR_FRAME_SIZE:]
            else:
                buf = buf[idx + 1:]


def lidar_point_process_thread():
    global temporal_cloud

    if not USE_LIDAR:
        return

    point_dt = 1.0 / LIDAR_POINT_RATE_HZ
    imu_miss = 0

    while running:
        try:
            frame_time, frame = lidar_frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        start_angle = struct.unpack_from("<H", frame, 4)[0] / 100.0
        end_angle = struct.unpack_from("<H", frame, 42)[0] / 100.0
        if end_angle < start_angle:
            end_angle += 360.0

        raw = np.frombuffer(frame[6:6 + LIDAR_POINTS_PER_FRAME * 3], dtype=np.uint8)
        raw = raw.reshape(LIDAR_POINTS_PER_FRAME, 3)

        dists = raw[:, 0].astype(np.uint16) | (raw[:, 1].astype(np.uint16) << 8)
        valid = np.where(dists > 0)[0]
        if len(valid) == 0:
            continue

        angles = start_angle + (end_angle - start_angle) * valid / (LIDAR_POINTS_PER_FRAME - 1)

        new_points = []
        now = time.monotonic()

        for pi, angle_deg in zip(valid, angles):
            if (pi % SPARSE_N) != 0:
                continue

            point_time = min(frame_time + pi * point_dt, time.monotonic())
            delta_yaw = get_delta_yaw(point_time)
            if delta_yaw is None:
                delta_yaw = 0.0
                imu_miss += 1

            corrected = normalize_angle(angle_deg - delta_yaw * YAW_ALPHA)
            rad = math.radians(corrected)
            dist = int(dists[pi])

            x = dist * math.sin(rad)
            y = dist * math.cos(rad)
            new_points.append((x, y, point_time))

        with cloud_lock:
            temporal_cloud.extend(new_points)
            cutoff = now - POINT_LIFETIME
            temporal_cloud = [p for p in temporal_cloud if p[2] >= cutoff]
            if len(temporal_cloud) > MAX_TEMPORAL_POINTS:
                temporal_cloud = temporal_cloud[-MAX_TEMPORAL_POINTS:]
            cloud_n = len(temporal_cloud)

        with state_lock:
            imu_hz = sensor_state["imu"]["hz"]

        set_state("lidar", ok=True, points=cloud_n, hz=imu_hz, miss=imu_miss)


# ==================================================================
# GPIOボタンによる手動ページ切り替え (任意)
# ==================================================================

def switch_thread(num_pages):
    """
    2方向ロッカースイッチ (GPIO17 = right, GPIO27 = left) でページを手動切替する。

    このタイプのスイッチは倒す動作の途中でもう一方の接点が一瞬反応する
    ことがあるため、以下のルールで「意図した操作」だけを確定させる。

      右へ操作 : 左が押された後、SWITCH_CONFIRM_SEC秒以上経過してから
                 右が押された場合にのみ確定する
      左へ操作 : 右が押された後、SWITCH_CONFIRM_SEC秒以上経過してから
                 左が押された場合にのみ確定する

    ボタンの状態はポーリング(is_pressed)で監視し、立ち上がりエッジ
    (押された瞬間)だけを操作トリガーとして扱う。
    """
    if not USE_BUTTON:
        return

    try:
        right_btn = Button(RIGHT_PIN, pull_up=True)
        left_btn = Button(LEFT_PIN, pull_up=True)
    except Exception as e:
        print("[SWITCH] 初期化に失敗しました:", e)
        return

    print(f"[SWITCH] 右=GPIO{RIGHT_PIN} 左=GPIO{LEFT_PIN} "
          f"(確定判定 {SWITCH_CONFIRM_SEC}秒)")

    prev_right = False
    prev_left = False
    left_pressed_time = None   # 左が最後に「新しく」押された時刻
    right_pressed_time = None  # 右が最後に「新しく」押された時刻

    while running:
        now = time.monotonic()
        right_now = right_btn.is_pressed
        left_now = left_btn.is_pressed

        # 左が新たに押された(立ち上がりエッジ)
        if left_now and not prev_left:
            if (right_pressed_time is not None
                    and (now - right_pressed_time) >= SWITCH_CONFIRM_SEC):
                change_page(-1, num_pages)   # 左方向へページ送り
                right_pressed_time = None
            left_pressed_time = now

        # 右が新たに押された(立ち上がりエッジ)
        if right_now and not prev_right:
            if (left_pressed_time is not None
                    and (now - left_pressed_time) >= SWITCH_CONFIRM_SEC):
                change_page(+1, num_pages)   # 右方向へページ送り
                left_pressed_time = None
            right_pressed_time = now

        prev_left = left_now
        prev_right = right_now

        time.sleep(0.05)


# ==================================================================
# ラズパイ側/SSH端末のキーボードによる手動ページ切り替え (← → キー)
# ==================================================================

def keyboard_thread(num_pages):
    """
    ラズパイに直接つないだキーボード、またはVSCode Remote-SSHの
    ターミナルから矢印キー ← / → でページを手動切り替えする。
    A/D , H/L キーでも同様に操作可能。q キーでプログラムを終了する。

    ※ SSH等の端末(TTY)からの標準入力が前提。
      systemdサービス化するなど標準入力が無い環境では自動的に無効化される。
    """
    global running

    if not USE_KEYBOARD:
        return

    if not sys.stdin.isatty():
        print("[KEY] 標準入力が端末(TTY)ではないため、キーボード操作は無効です。")
        return

    fd = sys.stdin.fileno()
    try:
        old_settings = termios.tcgetattr(fd)
    except Exception as e:
        print("[KEY] 初期化に失敗しました:", e)
        return

    print("[KEY] キー操作: ← / → (または A/D, H/L) でページ切替, q で終了")

    try:
        tty.setcbreak(fd)

        while running:
            # 0.2秒だけ入力を待つ (block_thread_forever対策 & runningの定期チェック)
            ready, _, _ = select.select([sys.stdin], [], [], 0.2)
            if not ready:
                continue

            ch = sys.stdin.read(1)

            if ch == "\x1b":
                # 矢印キーは ESC + '[' + 'C'(右) / 'D'(左) の3バイトシーケンス
                seq = sys.stdin.read(2)
                if seq == "[C":       # →
                    change_page(+1, num_pages)
                elif seq == "[D":     # ←
                    change_page(-1, num_pages)

            elif ch in ("d", "D", "l", "L"):
                change_page(+1, num_pages)

            elif ch in ("a", "A", "h", "H"):
                change_page(-1, num_pages)

            elif ch in ("q", "Q"):
                running = False

    except Exception as e:
        print("[KEY] エラー:", e)

    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass


# ==================================================================
# ゲームコントローラ (evdev) による高度な操作
#   - 左スティック 左右 : ページ切り替え (倒しきったら1回だけ発火)
#   - 右スティック       : LiDAR描画距離(ズーム)を連続的に調整
#   - ボタン             : ページ直接ジャンプ / 自動巡回ON-OFF / ズームリセット
# ==================================================================

def find_controller_device():
    """CONTROLLER_DEVICEが指定されていればそれを使い、
    無指定の場合はEV_ABS(スティック)とEV_KEY(ボタン)を持つ
    最初のデバイスを自動的に探して使う。"""

    if CONTROLLER_DEVICE:
        try:
            return InputDevice(CONTROLLER_DEVICE)
        except Exception as e:
            print("[PAD] 指定デバイスを開けませんでした:", CONTROLLER_DEVICE, e)
            return None

    for path in list_devices():
        try:
            dev = InputDevice(path)
            caps = dev.capabilities()
            if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
                print(f"[PAD] コントローラを自動検出しました: {dev.path} ({dev.name})")
                return dev
        except Exception:
            continue

    return None


def build_controller_button_actions(num_pages):
    """
    ボタンコード -> 動作の対応表を作る。

    実機のボタンコードはコントローラの機種によって異なる。
    ここに無いボタンを押すと未割当としてコンソールにコードが
    表示されるので、それを見ながら下の mapping に自由に追加できる。
    """

    def jump_to_page(title):
        for i, (t, _fn) in enumerate(PAGE_DEFS):
            if t == title:
                set_page_index(i, num_pages)
                return

    mapping = {
        "BTN_SOUTH": lambda: change_page(+1, num_pages),     # A相当 : 次のページ
        "BTN_EAST":  lambda: change_page(-1, num_pages),     # B相当 : 前のページ
        "BTN_TR":    lambda: jump_to_page("LIDAR"),           # R1相当: LiDARページへ直接ジャンプ
        "BTN_TL":    lambda: jump_to_page("DASHBOARD"),       # L1相当: ダッシュボードへ直接ジャンプ
        "BTN_START": toggle_auto_rotate,                       # STARTボタン : 自動巡回ON/OFF
        "BTN_SELECT": reset_lidar_range,                       # SELECT/BACK: LiDARズームをリセット
        "BTN_THUMBR": reset_lidar_range,                       # 右スティック押し込みでも同様
    }

    actions = {}
    for name, fn in mapping.items():
        code = getattr(ecodes, name, None)
        if code is not None:
            actions[code] = fn

    return actions


def controller_thread(num_pages):
    if not USE_CONTROLLER:
        return

    dev = find_controller_device()
    if dev is None:
        print("[PAD] コントローラが見つかりませんでした。ENABLE_CONTROLLERをFalseにするか、"
              "CONTROLLER_DEVICEのパスを確認してください。")
        return

    print(f"[PAD] 接続しました: {dev.name} ({dev.path})")
    print("[PAD] 左スティック 左右=ページ切替 / 右スティック=LiDAR描画距離ズーム")

    button_actions = build_controller_button_actions(num_pages)

    left_x = 0
    right_x = 0
    right_y = 0
    left_returned_to_center = True
    last_tick = time.monotonic()

    while running:
        try:
            ready, _, _ = select.select([dev.fd], [], [], 0.05)
        except Exception as e:
            print("[PAD] コントローラが切断されました:", e)
            break

        if ready:
            try:
                events = list(dev.read())
            except Exception as e:
                print("[PAD] 読み取りエラー:", e)
                break

            for event in events:

                if event.type == ecodes.EV_KEY:
                    if event.value == 1:   # 押された瞬間のみ反応(離す/長押しリピートは無視)
                        action = button_actions.get(event.code)
                        if action:
                            action()
                        else:
                            print(f"[PAD] 未割当ボタン code={event.code} "
                                  f"(build_controller_button_actionsに追加できます)")

                elif event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        left_x = event.value
                    elif event.code == ecodes.ABS_RX:
                        right_x = event.value
                    elif event.code == ecodes.ABS_RY:
                        right_y = event.value

        # --- 左スティック 左右 : ページ切替 (倒しきった瞬間に1回だけ発火) ---
        if left_x > STICK_PAGE_THRESHOLD and left_returned_to_center:
            change_page(+1, num_pages)
            left_returned_to_center = False
        elif left_x < -STICK_PAGE_THRESHOLD and left_returned_to_center:
            change_page(-1, num_pages)
            left_returned_to_center = False
        elif abs(left_x) < CONTROLLER_DEADZONE:
            left_returned_to_center = True

        # --- 右スティック : LiDAR描画距離を連続的にズーム ---
        now = time.monotonic()
        dt = now - last_tick
        last_tick = now

        axis_val = right_x if abs(right_x) > abs(right_y) else -right_y
        if abs(axis_val) >= CONTROLLER_DEADZONE:
            ratio = max(-1.0, min(1.0, axis_val / CONTROLLER_AXIS_MAX))
            adjust_lidar_range(ratio * LIDAR_RANGE_STEP_PER_SEC * dt)


# ==================================================================
# 描画関連 : カラーパレット / フォント
# ==================================================================

COL_BG = (16, 17, 22)
COL_CARD = (28, 30, 38)
COL_CARD_BORDER = (52, 55, 66)
COL_ACCENT = (64, 180, 255)
COL_ACCENT2 = (255, 145, 70)
COL_OK = (90, 220, 130)
COL_WARN = (250, 200, 60)
COL_ERROR = (255, 90, 90)
COL_TEXT = (235, 236, 240)
COL_SUBTEXT = (150, 152, 165)
COL_GRID = (40, 42, 52)


def load_font(size, bold=False):
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


FONTS = {
    "title": load_font(16, bold=True),
    "label": load_font(12),
    "value": load_font(20, bold=True),
    "small": load_font(10),
    "tiny": load_font(9),
}


# ==================================================================
# 描画ヘルパー (モダン風パーツ)
# ==================================================================

def draw_card(draw, x, y, w, h, title=None):
    draw.rounded_rectangle([x, y, x + w, y + h], radius=8,
                            fill=COL_CARD, outline=COL_CARD_BORDER, width=1)
    if title:
        draw.text((x + 8, y + 6), title, font=FONTS["small"], fill=COL_SUBTEXT)


def draw_bar(draw, x, y, w, h, ratio, color, bg=COL_GRID):
    ratio = max(0.0, min(1.0, ratio))
    draw.rounded_rectangle([x, y, x + w, y + h], radius=3, fill=bg)
    fw = int(w * ratio)
    if fw > 3:
        draw.rounded_rectangle([x, y, x + fw, y + h], radius=3, fill=color)


def draw_status_dot(draw, x, y, ok, radius=4):
    color = COL_OK if ok else COL_ERROR
    draw.ellipse([x - radius, y - radius, x + radius, y + radius], fill=color)


def draw_header(draw, title, state):
    draw.rectangle([0, 0, CANVAS_W, 26], fill=(22, 24, 30))
    draw.text((10, 5), title, font=FONTS["title"], fill=COL_TEXT)

    dots = [
        ("I", state["imu"]["ok"]),
        ("C", state["color0"]["ok"] or state["color1"]["ok"]),
        ("D", state["dist2"]["ok"] or state["dist3"]["ok"]),
        ("L", state["lidar"]["ok"]),
    ]
    dx = CANVAS_W - 12
    for label, ok in reversed(dots):
        draw_status_dot(draw, dx, 13, ok)
        dx -= 16


def draw_page_dots(draw, page_idx, num_pages):
    total_w = num_pages * 14
    start_x = (CANVAS_W - total_w) // 2 + 7
    y = CANVAS_H - 10
    for i in range(num_pages):
        color = COL_ACCENT if i == page_idx else COL_GRID
        r = 3
        cx = start_x + i * 14
        draw.ellipse([cx - r, y - r, cx + r, y + r], fill=color)


def draw_attitude_indicator(draw, cx, cy, r, roll_deg, pitch_deg):
    """簡易姿勢インジケータ (水平線がロール/ピッチに応じて動く)"""
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=COL_CARD,
                 outline=COL_ACCENT, width=2)

    roll = math.radians(roll_deg)
    pitch_offset = max(-r * 0.8, min(r * 0.8, pitch_deg * (r / 45.0)))

    line_len = r * 1.6
    dx = math.cos(roll) * line_len / 2
    dy = math.sin(roll) * line_len / 2
    ox = math.sin(roll) * pitch_offset
    oy = -math.cos(roll) * pitch_offset

    x1, y1 = cx - dx + ox, cy - dy + oy
    x2, y2 = cx + dx + ox, cy + dy + oy
    draw.line([x1, y1, x2, y2], fill=COL_ACCENT2, width=3)

    # 機体固定マーカー(中心)
    draw.line([cx - 12, cy, cx - 4, cy], fill=COL_TEXT, width=2)
    draw.line([cx + 4, cy, cx + 12, cy], fill=COL_TEXT, width=2)
    draw.ellipse([cx - 2, cy - 2, cx + 2, cy + 2], fill=COL_TEXT)


def draw_compass(draw, cx, cy, r, yaw_deg):
    draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=COL_CARD,
                 outline=COL_ACCENT, width=2)

    for label, ang in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
        a = math.radians(ang - yaw_deg - 90)
        tx = cx + math.cos(a) * (r - 12)
        ty = cy + math.sin(a) * (r - 12)
        draw.text((tx, ty), label, font=FONTS["tiny"], fill=COL_SUBTEXT, anchor="mm")

    a = math.radians(-yaw_deg - 90)
    nx = cx + math.cos(a) * (r - 5)
    ny = cy + math.sin(a) * (r - 5)
    draw.line([cx, cy, nx, ny], fill=COL_ERROR, width=3)
    draw.ellipse([cx - 2, cy - 2, cx + 2, cy + 2], fill=COL_TEXT)


def rgbc_to_display_color(c, r, g, b):
    """TCS34725の生値を、画面表示用のおおよそのRGBに変換する"""
    if not c:
        return (40, 40, 45)
    scale = 255.0 / max(c, 1)
    rr = min(255, int(r * scale))
    gg = min(255, int(g * scale))
    bb = min(255, int(b * scale))
    return (rr, gg, bb)


def distance_color(mm):
    if mm is None:
        return COL_SUBTEXT
    if mm < 100:
        return COL_ERROR
    if mm < 400:
        return COL_WARN
    return COL_OK


# ==================================================================
# 各ページの描画関数
# ==================================================================

def render_dashboard(draw, state):
    pad = 6
    cw = (CANVAS_W - pad * 3) // 2
    ch = 78
    y0 = 32

    # --- IMUカード ---
    draw_card(draw, pad, y0, cw, ch, "IMU")
    if state["imu"]["ok"]:
        draw.text((pad + 8, y0 + 22), f"Yaw {state['imu']['yaw']:+.0f}",
                   font=FONTS["value"], fill=COL_TEXT)
        draw.text((pad + 8, y0 + 50),
                   f"R{state['imu']['roll']:+.0f} P{state['imu']['pitch']:+.0f}",
                   font=FONTS["small"], fill=COL_SUBTEXT)
    else:
        draw.text((pad + 8, y0 + 34), "NO DATA", font=FONTS["label"], fill=COL_ERROR)

    # --- カラーカード ---
    x1 = pad * 2 + cw
    draw_card(draw, x1, y0, cw, ch, "COLOR")
    if state["color0"]["ok"] or state["color1"]["ok"]:
        sw = 22
        col0 = rgbc_to_display_color(**{k: state["color0"][k] for k in ("c", "r", "g", "b")})
        col1 = rgbc_to_display_color(**{k: state["color1"][k] for k in ("c", "r", "g", "b")})
        draw.rounded_rectangle([x1 + 10, y0 + 26, x1 + 10 + sw, y0 + 26 + sw], radius=4, fill=col0)
        draw.rounded_rectangle([x1 + 40, y0 + 26, x1 + 40 + sw, y0 + 26 + sw], radius=4, fill=col1)
        draw.text((x1 + 8, y0 + 54), "CH0 / CH1", font=FONTS["tiny"], fill=COL_SUBTEXT)
    else:
        draw.text((x1 + 8, y0 + 34), "NO DATA", font=FONTS["label"], fill=COL_ERROR)

    y1 = y0 + ch + pad

    # --- 距離カード ---
    draw_card(draw, pad, y1, cw, ch, "DISTANCE")
    d2 = state["dist2"]["mm"]
    d3 = state["dist3"]["mm"]
    if state["dist2"]["ok"] or state["dist3"]["ok"]:
        d2s = f"{d2:.0f}mm" if d2 is not None else "--"
        d3s = f"{d3:.0f}mm" if d3 is not None else "--"
        draw.text((pad + 8, y1 + 24), f"CH2 {d2s}", font=FONTS["label"],
                   fill=distance_color(d2))
        draw.text((pad + 8, y1 + 46), f"CH3 {d3s}", font=FONTS["label"],
                   fill=distance_color(d3))
    else:
        draw.text((pad + 8, y1 + 34), "NO DATA", font=FONTS["label"], fill=COL_ERROR)

    # --- LiDARカード ---
    draw_card(draw, x1, y1, cw, ch, "LIDAR")
    if state["lidar"]["ok"]:
        draw.text((x1 + 8, y1 + 22), f"{state['lidar']['points']} pts",
                   font=FONTS["value"], fill=COL_TEXT)
        draw.text((x1 + 8, y1 + 50), f"IMU {state['lidar']['hz']:.0f}Hz",
                   font=FONTS["small"], fill=COL_SUBTEXT)
    else:
        draw.text((x1 + 8, y1 + 34), "NO DATA", font=FONTS["label"], fill=COL_ERROR)


def render_imu(draw, state):
    y0 = 34
    if not state["imu"]["ok"]:
        draw.text((CANVAS_W // 2, CANVAS_H // 2), "NO DATA", font=FONTS["value"],
                   fill=COL_ERROR, anchor="mm")
        return

    draw_attitude_indicator(draw, 62, y0 + 50, 46,
                             state["imu"]["roll"], state["imu"]["pitch"])
    draw_compass(draw, CANVAS_W - 62, y0 + 50, 46, state["imu"]["yaw"])

    draw.text((20, y0 + 100), f"R {state['imu']['roll']:+.1f}", font=FONTS["label"], fill=COL_TEXT)
    draw.text((100, y0 + 100), f"P {state['imu']['pitch']:+.1f}", font=FONTS["label"], fill=COL_TEXT)
    draw.text((180, y0 + 100), f"Y {state['imu']['yaw']:+.1f}", font=FONTS["label"], fill=COL_TEXT)

    # 加速度バー
    ay0 = y0 + 122
    accel = (("Ax", state["imu"]["ax"]), ("Ay", state["imu"]["ay"]), ("Az", state["imu"]["az"]))
    for i, (label, val) in enumerate(accel):
        yy = ay0 + i * 20
        draw.text((10, yy), label, font=FONTS["small"], fill=COL_SUBTEXT)
        ratio = max(0.0, min(1.0, (val + 10) / 20.0))
        draw_bar(draw, 40, yy + 2, 150, 10, ratio, COL_ACCENT)
        draw.text((195, yy), f"{val:+.1f}", font=FONTS["tiny"], fill=COL_TEXT)


def render_color(draw, state):
    y0 = 34
    pad = 10
    box_w = (CANVAS_W - pad * 3) // 2
    box_h = 110

    for i, key in enumerate(("color0", "color1")):
        x = pad + i * (box_w + pad)
        draw_card(draw, x, y0, box_w, box_h, f"CH{i} TCS34725")
        s = state[key]
        if s["ok"]:
            col = rgbc_to_display_color(s["c"], s["r"], s["g"], s["b"])
            draw.rounded_rectangle([x + 10, y0 + 22, x + box_w - 10, y0 + 62],
                                    radius=6, fill=col, outline=COL_CARD_BORDER)
            draw.text((x + 10, y0 + 68), f"C {s['c']}", font=FONTS["tiny"], fill=COL_SUBTEXT)
            draw.text((x + 10, y0 + 82), f"R {s['r']}", font=FONTS["tiny"], fill=(255, 120, 120))
            draw.text((x + 10, y0 + 94), f"G {s['g']}", font=FONTS["tiny"], fill=(120, 255, 120))
            draw.text((x + box_w // 2, y0 + 94), f"B {s['b']}", font=FONTS["tiny"], fill=(120, 170, 255))
        else:
            draw.text((x + box_w // 2, y0 + box_h // 2), "NO DATA",
                       font=FONTS["small"], fill=COL_ERROR, anchor="mm")


def render_distance(draw, state):
    y0 = 34
    pad = 20
    gauge_w = 60
    gauge_h = 140
    max_mm = 2000.0

    for i, key in enumerate(("dist2", "dist3")):
        x = pad + i * (CANVAS_W // 2)
        s = state[key]
        draw.text((x, y0), f"CH{i + 2} VL53L1X", font=FONTS["small"], fill=COL_SUBTEXT)

        gx = x
        gy = y0 + 18
        draw.rounded_rectangle([gx, gy, gx + gauge_w, gy + gauge_h], radius=8,
                                fill=COL_CARD, outline=COL_CARD_BORDER, width=1)

        if s["ok"] and s["mm"] is not None:
            ratio = max(0.0, min(1.0, s["mm"] / max_mm))
            fh = int(gauge_h * ratio)
            color = distance_color(s["mm"])
            draw.rounded_rectangle(
                [gx, gy + gauge_h - fh, gx + gauge_w, gy + gauge_h],
                radius=8, fill=color)
            draw.text((gx, gy + gauge_h + 6), f"{s['mm']:.0f}mm",
                       font=FONTS["label"], fill=COL_TEXT)
        else:
            draw.text((gx, gy + gauge_h + 6), "NO DATA",
                       font=FONTS["tiny"], fill=COL_ERROR)


def render_lidar(draw, state):
    with settings_lock:
        draw_range = runtime_settings["draw_range_mm"]

    cx, cy = CANVAS_W // 2, CANVAS_H // 2 + 6
    pix = (CANVAS_W * 0.42) / draw_range

    with cloud_lock:
        cloud = list(temporal_cloud)

    now = time.monotonic()

    if len(cloud) > LIDAR_MAX_DRAW_POINTS:
        step = len(cloud) // LIDAR_MAX_DRAW_POINTS
        cloud = cloud[::max(step, 1)]

    for x, y, ts in cloud:
        age = now - ts
        brightness = max(0.0, min(1.0, 1.0 - age / POINT_LIFETIME))
        if brightness <= 0.05:
            continue
        px = int(cx + x * pix)
        py = int(cy - y * pix)
        if 0 <= px < CANVAS_W and 26 <= py < CANVAS_H - 14:
            g = int(80 + brightness * 175)
            draw.ellipse([px - 1, py - 1, px + 1, py + 1], fill=(0, g, 0))

    # 中心 & 前方マーカー
    draw.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=COL_ERROR)
    draw.line([cx, cy, cx, cy - 40], fill=COL_ACCENT2, width=2)

    draw.text((6, 30), f"P {state['lidar']['points']}", font=FONTS["tiny"], fill=COL_SUBTEXT)
    draw.text((6, 42), f"IMU {state['lidar']['hz']:.0f}Hz", font=FONTS["tiny"], fill=COL_SUBTEXT)
    draw.text((6, 54), f"Range {draw_range:.0f}mm", font=FONTS["tiny"], fill=COL_SUBTEXT)

    if not state["lidar"]["ok"]:
        draw.text((CANVAS_W // 2, CANVAS_H // 2), "NO DATA", font=FONTS["value"],
                   fill=COL_ERROR, anchor="mm")


PAGE_DEFS = [
    ("DASHBOARD", render_dashboard),
    ("IMU", render_imu),
    ("COLOR", render_color),
    ("DISTANCE", render_distance),
]
if USE_LIDAR:
    PAGE_DEFS.append(("LIDAR", render_lidar))


# ==================================================================
# 描画スレッド (メインループ)
# ==================================================================

def init_display():
    """
    viewer.py の Viewer を起動し、Windows側のブラウザから
    http://<ラズパイのIP>:8000/ でGUIを見られるようにする。
    """
    v = viewer.Viewer(host=VIEWER_HOST, port=VIEWER_PORT, refresh_ms=VIEWER_REFRESH_MS)
    v.start()
    print(f"[VIEWER] http://<ラズパイのIP>:{VIEWER_PORT}/ をブラウザで開いてください")
    return v


def draw_thread(disp):
    global current_page_idx

    last_switch = time.monotonic()
    prev_manual_switch = last_manual_switch
    frame_interval = 1.0 / DISPLAY_FPS

    while running:
        loop_start = time.monotonic()

        with page_lock:
            manual_ts = last_manual_switch

        if manual_ts != prev_manual_switch:
            # キーボード/ボタンで手動切替があった -> 自動巡回のタイマーをリセット
            prev_manual_switch = manual_ts
            last_switch = loop_start

        since_manual = loop_start - manual_ts

        with settings_lock:
            auto_rotate_on = runtime_settings["auto_rotate"]

        if auto_rotate_on and since_manual >= MANUAL_HOLD_SEC \
                and (loop_start - last_switch) >= PAGE_DURATION_SEC:
            with page_lock:
                current_page_idx = (current_page_idx + 1) % len(PAGE_DEFS)
            last_switch = loop_start

        img = Image.new("RGB", (CANVAS_W, CANVAS_H), COL_BG)
        draw = ImageDraw.Draw(img)

        state = get_state_snapshot()

        with page_lock:
            idx = current_page_idx
        title, render_fn = PAGE_DEFS[idx]

        render_fn(draw, state)
        draw_header(draw, title, state)
        draw_page_dots(draw, idx, len(PAGE_DEFS))

        if disp is not None:
            try:
                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=JPEG_QUALITY)
                disp.update(VIEWER_IMAGE_NAME, buf.getvalue())
            except Exception as e:
                print("[VIEWER] 更新エラー:", e)

        elapsed = time.monotonic() - loop_start
        sleep_time = frame_interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


# ==================================================================
# 終了処理
# ==================================================================

def shutdown(disp):
    print("終了処理中...")
    if disp is not None:
        try:
            disp.stop()
        except Exception:
            pass


def handle_signal(signum, frame):
    global running
    running = False


# ==================================================================
# メイン
# ==================================================================

def main():
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    print("======================================================")
    print(" 全センサー統合ダッシュボード起動")
    print(f"   IMU(BNO085)      : {'有効' if USE_IMU else '無効'}")
    print(f"   COLOR(TCS34725)  : {'有効' if USE_TCA_COLOR else '無効'}")
    print(f"   DISTANCE(VL53L1X): {'有効' if USE_TCA_DISTANCE else '無効'}")
    print(f"   LIDAR(MS200)     : {'有効' if USE_LIDAR else '無効'}")
    print(f"   SWITCH(右/左)     : {'有効' if USE_BUTTON else '無効'}")
    print(f"   KEYBOARD         : {'有効' if USE_KEYBOARD else '無効'}")
    print(f"   CONTROLLER(evdev): {'有効' if USE_CONTROLLER else '無効'}")
    print("======================================================")

    disp = init_display()

    threads = []
    threads.append(threading.Thread(target=imu_thread, daemon=True))
    threads.append(threading.Thread(target=tca_color_thread, daemon=True))
    threads.append(threading.Thread(target=tca_distance_thread, daemon=True))

    if USE_LIDAR:
        threads.append(threading.Thread(target=lidar_reader_thread, daemon=True))
        threads.append(threading.Thread(target=lidar_parser_thread, daemon=True))
        threads.append(threading.Thread(target=lidar_point_process_thread, daemon=True))

    if USE_BUTTON:
        threads.append(threading.Thread(target=switch_thread, args=(len(PAGE_DEFS),), daemon=True))

    if USE_KEYBOARD:
        threads.append(threading.Thread(target=keyboard_thread, args=(len(PAGE_DEFS),), daemon=True))

    if USE_CONTROLLER:
        threads.append(threading.Thread(target=controller_thread, args=(len(PAGE_DEFS),), daemon=True))

    for t in threads:
        t.start()

    try:
        draw_thread(disp)
    finally:
        shutdown(disp)


if __name__ == "__main__":
    main()