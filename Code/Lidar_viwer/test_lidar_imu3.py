"""
lidar_imu_viz.py  ―  ORBBEC MS200 + BNO085  2D Temporal LiDAR Viewer
Raspberry Pi 5  |  Python 3.11+
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
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GYROSCOPE

# ===========================================================
# CONFIG  ―  ここだけ触れば動作を調整できる
# ===========================================================

# ---------- LiDAR ----------
LIDAR_PORT   = "/dev/ttyAMA2"
LIDAR_BAUD   = 230400
FRAME_SIZE   = 47
HEADER       = 0x54
POINTS_PER_FRAME = 12
POINT_RATE_HZ    = 4500          # MS200 公称レート

# ---------- Temporal Cloud ----------
POINT_LIFETIME      = 0.5       # 点の寿命 [sec]
MAX_TEMPORAL_POINTS = 5000

# ---------- IMU 補正 ----------
# YAW_ALPHA : LiDAR角度にかけるIMUヨー補正強度 (0.0=補正なし / 1.0=フル補正)
YAW_ALPHA       = 0.35
# GYRO_PREDICT_DT : ジャイロによる未来予測時間 [sec]  0.0=予測なし
GYRO_PREDICT_DT = 0.015
# IMU_COMPLEMENT  : ジャイロ積分とクォータニオンのブレンド比 (0.0=クォータニオンのみ)
IMU_COMPLEMENT  = 0.05

# ---------- Sparse Sampling ----------
# SPARSE_N=1 で全点使用  2以上でN点に1点使用（CPU負荷軽減）
SPARSE_N = 2

# ---------- Draw ----------
WIDTH  = 1200
HEIGHT = 1200
CX     = WIDTH  // 2
CY     = HEIGHT // 2
DRAW_RANGE_MM = 1000.0           # 半径1m表示
PIX = WIDTH / (DRAW_RANGE_MM * 2)
DRAW_FPS_LIMIT = 30              # 描画上限フレームレート
POINT_RADIUS   = 3               # 点群の描画半径 [px]（1=1px dot, 2以上=circle）

# ---------- Debug OSD ----------
SHOW_OSD = True                  # デバッグ情報をオーバーレイ表示

# ===========================================================
# LOGGING
# ===========================================================

logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s [%(threadName)s] %(levelname)s: %(message)s"
)
log = logging.getLogger("lidar")

# ===========================================================
# CRC8 TABLE (LD/MS200 共通)
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

def crc8(frame: bytes | bytearray) -> int:
    crc = 0
    for b in frame:
        crc = CRC8_TABLE[crc ^ b]
    return crc

# ===========================================================
# QUEUES
# ===========================================================

raw_queue   = queue.Queue(maxsize=8)
frame_queue = queue.Queue(maxsize=16)

# ===========================================================
# SHARED STATE
# ===========================================================

imu_lock   = threading.Lock()
imu_buffer = deque(maxlen=1024)   # (ts, yaw_deg, gz_deg_s)

cloud_lock     = threading.Lock()
temporal_cloud = []               # list of (x, y, ts)

# デバッグ統計（atomicに更新）
_stats_lock       = threading.Lock()
_stats = {
    "imu_hz":       0.0,
    "lidar_fps":    0.0,
    "draw_fps":     0.0,
    "q_raw":        0,
    "q_frame":      0,
    "cloud_n":      0,
    "imu_miss":     0,   # 補間失敗カウント
    "crc_err":      0,   # CRCエラーカウント
    "yaw_now":      0.0,
}

frame_counter = 0

# ===========================================================
# UTILS
# ===========================================================

def safe_put(q: queue.Queue, item):
    """キューが満杯のとき古いエントリを捨てて入れる"""
    if q.full():
        try:
            q.get_nowait()
        except queue.Empty:
            pass
    try:
        q.put_nowait(item)
    except queue.Full:
        pass

def normalize_angle(a: float) -> float:
    while a >  180: a -= 360
    while a < -180: a += 360
    return a

def angle_diff(a: float, b: float) -> float:
    d = a - b
    if d >  180: d -= 360
    if d < -180: d += 360
    return d

def update_stat(**kw):
    with _stats_lock:
        _stats.update(kw)

# ===========================================================
# QUATERNION → EULER
# ===========================================================

def quat_to_euler(qi, qj, qk, qr):
    """BNO085クォータニオン → ロボット座標系のroll/pitch/yaw [deg]"""
    x, y, z, w = qj, -qi, qk, qr

    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2 * (x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(-roll), math.degrees(pitch), math.degrees(-yaw)

# ===========================================================
# IMU THREAD
# ===========================================================

def imu_thread():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)

    ts_prev = time.monotonic()
    hz_alpha = 0.05

    while True:
        try:
            qi, qj, qk, qr = bno.quaternion
            _, _, yaw = quat_to_euler(qi, qj, qk, qr)
            gx, gy, gz = bno.gyro          # [rad/s]

            yaw = normalize_angle(yaw)
            ts  = time.monotonic()

            with imu_lock:
                imu_buffer.append((ts, yaw, math.degrees(gz)))

            # IMU Hz 推定
            dt = ts - ts_prev
            ts_prev = ts
            if dt > 0:
                hz_now = 1.0 / dt
                with _stats_lock:
                    _stats["imu_hz"] = (
                        (1 - hz_alpha) * _stats["imu_hz"] + hz_alpha * hz_now
                    )
                    _stats["yaw_now"] = yaw

        except Exception as e:
            log.debug("IMU read error: %s", e)

        time.sleep(0.004)   # ~200Hz 上限

# ===========================================================
# IMU INTERPOLATION
# ===========================================================

def get_interpolated_yaw(target_time: float):
    """
    target_time に最も近いIMU補間ヨー角を返す。
    ジャイロ微分予測 (GYRO_PREDICT_DT) を加算して遅延を補う。
    失敗時は None。
    """
    with imu_lock:
        if len(imu_buffer) < 2:
            return None
        data = list(imu_buffer)

    # 二分探索で挿入位置を求める
    lo, hi = 0, len(data) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if data[mid][0] <= target_time:
            lo = mid
        else:
            hi = mid

    t0, y0, g0 = data[lo]
    t1, y1, g1 = data[hi]

    if t0 <= target_time <= t1:
        ratio      = (target_time - t0) / (t1 - t0 + 1e-9)
        dyaw       = angle_diff(y1, y0)
        interp_yaw = y0 + dyaw * ratio
        interp_gz  = g0 + (g1 - g0) * ratio
    else:
        # target_time がバッファ外（最新より未来）
        interp_yaw = data[-1][1]
        interp_gz  = data[-1][2]

    # ジャイロ予測
    pred_yaw = normalize_angle(interp_yaw + interp_gz * GYRO_PREDICT_DT)
    return pred_yaw

# ===========================================================
# LIDAR RX THREAD
# ===========================================================

def lidar_reader_thread():
    ser = serial.Serial(LIDAR_PORT, LIDAR_BAUD, timeout=0.01)
    # MS200 開始コマンド
    ser.write(b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2")
    log.info("LiDAR reader started")

    while True:
        try:
            data = ser.read(4096)
            if data:
                safe_put(raw_queue, (time.monotonic(), data))
        except Exception as e:
            log.warning("Serial read error: %s", e)
            time.sleep(0.1)

# ===========================================================
# PARSER THREAD
# ===========================================================

def parser_thread():
    global frame_counter
    buf = bytearray()
    alpha = 0.05
    lidar_fps = 0.0
    last_ts   = time.monotonic()
    crc_errs  = 0

    while True:
        # キューに溜まった分を一括取得（最大バースト消化）
        try:
            recv_ts, data = raw_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        while True:
            try:
                recv_ts, data = raw_queue.get_nowait()
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

            frame = buf[idx : idx + FRAME_SIZE]
            expected_crc = crc8(frame[:-1])

            if expected_crc == frame[-1]:
                ts_now = time.monotonic()
                safe_put(frame_queue, (ts_now, frame_counter, bytes(frame)))
                frame_counter += 1

                # LiDAR FPS 推定
                dt = ts_now - last_ts
                last_ts = ts_now
                if dt > 0:
                    fps_inst = 1.0 / dt
                    lidar_fps = (1 - alpha) * lidar_fps + alpha * fps_inst
                    with _stats_lock:
                        _stats["lidar_fps"] = lidar_fps
                        _stats["crc_err"]   = crc_errs

                buf = buf[idx + FRAME_SIZE:]
            else:
                crc_errs += 1
                buf = buf[idx + 1:]

        with _stats_lock:
            _stats["q_raw"]   = raw_queue.qsize()
            _stats["q_frame"] = frame_queue.qsize()

# ===========================================================
# POINT PROCESS THREAD
# ===========================================================

def point_process_thread():
    global temporal_cloud

    POINT_DT = 1.0 / POINT_RATE_HZ
    imu_miss = 0

    while True:
        try:
            frame_time, frame_id, frame = frame_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        # --- アングル解析 ---
        start_angle = struct.unpack_from("<H", frame, 4)[0] / 100.0
        end_angle   = struct.unpack_from("<H", frame, 42)[0] / 100.0
        if end_angle < start_angle:
            end_angle += 360.0

        # --- 距離データ一括展開 (NumPy) ---
        raw = np.frombuffer(frame[6 : 6 + POINTS_PER_FRAME * 3], dtype=np.uint8)
        raw = raw.reshape(POINTS_PER_FRAME, 3)
        dists = (raw[:, 0].astype(np.uint16) | (raw[:, 1].astype(np.uint16) << 8))

        valid_mask = dists > 0
        valid_idx  = np.where(valid_mask)[0]

        if len(valid_idx) == 0:
            continue

        # Sparse sampling
        offset      = frame_id % SPARSE_N
        sparse_mask = ((valid_idx + offset) % SPARSE_N) == 0
        sel_idx     = valid_idx[sparse_mask]

        if len(sel_idx) == 0:
            continue

        # 各点の角度
        angles_deg = (
            start_angle
            + (end_angle - start_angle) * sel_idx / (POINTS_PER_FRAME - 1)
        )

        now = time.monotonic()
        new_points = []

        for i, (pi, angle_deg) in enumerate(zip(sel_idx, angles_deg)):
            point_time = frame_time + pi * POINT_DT

            imu_yaw = get_interpolated_yaw(point_time)
            if imu_yaw is None:
                imu_miss += 1
                continue

            # 補正角度：YAW_ALPHA で強度を調整
            corrected = normalize_angle(angle_deg - imu_yaw * YAW_ALPHA)
            rad = math.radians(corrected)

            dist = int(dists[pi])
            x = dist * math.sin(rad)
            y = dist * math.cos(rad)

            new_points.append((x, y, point_time))

        # --- Temporal Cloud 更新 ---
        with cloud_lock:
            temporal_cloud.extend(new_points)
            cutoff = now - POINT_LIFETIME
            temporal_cloud = [p for p in temporal_cloud if p[2] >= cutoff]
            if len(temporal_cloud) > MAX_TEMPORAL_POINTS:
                temporal_cloud = temporal_cloud[-MAX_TEMPORAL_POINTS:]
            cloud_n = len(temporal_cloud)

        with _stats_lock:
            _stats["imu_miss"] = imu_miss
            _stats["cloud_n"]  = cloud_n

# ===========================================================
# DRAW THREAD  ―  NumPy vectorized描画
# ===========================================================

def draw_thread():
    cv2.namedWindow("MS200 LiDAR", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("MS200 LiDAR", WIDTH, HEIGHT)

    frame_interval = 1.0 / DRAW_FPS_LIMIT
    last_draw = time.monotonic()
    alpha_fps = 0.1
    draw_fps  = 0.0

    while True:
        # FPS 上限
        now = time.monotonic()
        wait = frame_interval - (now - last_draw)
        if wait > 0:
            time.sleep(wait)

        now = time.monotonic()
        dt  = now - last_draw
        last_draw = now
        if dt > 0:
            draw_fps = (1 - alpha_fps) * draw_fps + alpha_fps / dt

        # --- スナップショット取得 ---
        with cloud_lock:
            cloud = list(temporal_cloud)

        img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        # --- Vectorized 描画 ---
        if cloud:
            arr = np.array(cloud, dtype=np.float32)   # (N, 3)
            xs  = arr[:, 0]
            ys  = arr[:, 1]
            tss = arr[:, 2]

            ages    = now - tss
            weights = np.clip(1.0 - ages / POINT_LIFETIME, 0, 1)

            px = (CX + xs * PIX).astype(np.int32)
            py = (CY - ys * PIX).astype(np.int32)

            in_bounds = (
                (px >= POINT_RADIUS) & (px < WIDTH  - POINT_RADIUS) &
                (py >= POINT_RADIUS) & (py < HEIGHT - POINT_RADIUS)
            )
            px = px[in_bounds]
            py = py[in_bounds]
            bv = (weights[in_bounds] * 255).astype(np.uint8)

            if POINT_RADIUS <= 1:
                # 1px: np.maximum.at で最大輝度保持（高速）
                np.maximum.at(img[:, :, 1], (py, px), bv)
            else:
                # 2px以上: 各点を塗りつぶし円で描画
                # 同一ピクセル重複を避けるため輝度でソート（明るい点が上書き）
                order = np.argsort(bv)
                for i in order:
                    cv2.circle(
                        img,
                        (int(px[i]), int(py[i])),
                        POINT_RADIUS,
                        (0, int(bv[i]), 0),
                        -1,
                        cv2.LINE_AA,
                    )

        # --- オーバーレイ ---
        # 中心マーカー
        cv2.circle(img, (CX, CY), 4, (0, 0, 255), -1)
        # 前方軸
        cv2.line(img, (CX, CY), (CX, CY - 80), (255, 80, 0), 2)

        if SHOW_OSD:
            with _stats_lock:
                s = dict(_stats)
            s["draw_fps"] = draw_fps

            osd_lines = [
                f"Draw FPS : {s['draw_fps']:5.1f}",
                f"LiDAR FPS: {s['lidar_fps']:5.1f}",
                f"IMU  Hz  : {s['imu_hz']:5.1f}",
                f"Cloud N  : {s['cloud_n']:5d}",
                f"Yaw      : {s['yaw_now']:+7.2f} deg",
                f"YawAlpha : {YAW_ALPHA:.2f}",
                f"GyroPred : {GYRO_PREDICT_DT*1000:.1f} ms",
                f"IMU miss : {s['imu_miss']:5d}",
                f"CRC err  : {s['crc_err']:5d}",
                f"Q raw    : {s['q_raw']:3d}",
                f"Q frame  : {s['q_frame']:3d}",
            ]

            y0 = 28
            for i, line in enumerate(osd_lines):
                cv2.putText(
                    img, line,
                    (10, y0 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (200, 200, 200), 1, cv2.LINE_AA,
                )

        cv2.imshow("MS200 LiDAR", img)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:       # ESC で終了
            break
        elif key == ord('d'):
            # d キーでOSD切り替え（グローバル書き換え）
            import builtins
            builtins.__dict__   # noop  ←  実行時はグローバル変数で切り替え

    cv2.destroyAllWindows()

# ===========================================================
# MAIN
# ===========================================================

if __name__ == "__main__":
    threads = [
        threading.Thread(target=imu_thread,           name="IMU",     daemon=True),
        threading.Thread(target=lidar_reader_thread,  name="LiDAR-RX",daemon=True),
        threading.Thread(target=parser_thread,        name="Parser",  daemon=True),
        threading.Thread(target=point_process_thread, name="PtProc",  daemon=True),
    ]
    for t in threads:
        t.start()

    draw_thread()   # メインスレッドで実行（OpenCV必須）