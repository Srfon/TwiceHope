"""
lidar_drive.py
------------------------------------------------------------
LiDAR(ORBBEC MS200)の点群で壁に沿って走る。旋回は角度で確定させ、
前進中は左右の壁までの距離が等しくなるようにモーター量を調整する。

「壁あり/なし」は、ロボット中心から WALL_DIST_MM(=15cm)以内に
点があるかどうかで判定する。前 / 左 / 右の3方向を見て動きを決める。

動作ルール:
  ・前に壁が無い                      -> 前進(左右の距離を合わせながら)
  ・前+左が壁・右が空き               -> 右へ90度
  ・前+右が壁・左が空き               -> 左へ90度
  ・前だけ壁・左右とも空き            -> 左へ90度(左優先)
  ・前左右すべて壁(行き止まり)       -> 180度旋回
  旋回を始めたら、目標角度に達するまで他の判断はしない
  (= 旋回中に前進などへ切り替わらない)。

角度の測り方:
  IMU(BNO085)の yaw を使う。IMU が無い/失敗したときは時間ベース
  (TURN_DEG_PER_SEC を実測して設定)に自動で切り替わる。

停止について:
  ・ターミナルで Ctrl+C  -> 確実に停止(推奨)
  ・VSCode の停止ボタン(■)は SIGKILL のことが多く、ソフトでは止め
    られない。確実な非常停止は電源スイッチ。回りっぱなし時は stop.py。

構成:
  lidar_drive.py(これ) ── robot.py(Robot) ── servo_cli.py(STSBus)
"""

import time
import math
import signal
import threading
import serial
from collections import deque

from robot import Robot


# ============================================================
# 設定(自分の機体・配置に合わせて調整)
# ============================================================

# ---- LiDAR ----
LIDAR_PORT = "/dev/ttyAMA0"
LIDAR_BAUD = 230400

# LiDAR の「0度」が機体のどちらを向いているか(正面=0)。取り付けズレの補正。
FRONT_OFFSET_DEG = 0.0
# 左右が逆に動く場合は True(LiDAR の回転方向・取り付けの裏表で変わる)
MIRROR = False

# ---- 判定する範囲(角度) ----  正面=0、右=+、左=-
FRONT_HALF_DEG = 25.0          # 正面の扇形(±25°)
SIDE_CENTER_DEG = 90.0         # 左右は真横(±90°)を中心に見る
SIDE_HALF_DEG = 30.0           # 左右扇形の半分(90±30 = 60〜120°)

# ---- 判定する距離(mm) ----
WALL_DIST_MM = 150.0           # この距離内に点があれば「壁あり」。15cm。

# ---- 旋回 ----
TURN_SPEED = 1100              # 旋回の速さ
TURN90_DEG = 90.0              # 右左折で回る角度
TURN180_DEG = 180.0            # 行き止まりで回る角度
USE_IMU = True                 # True: IMU(BNO085)の角度で旋回を止める
TURN_DEG_PER_SEC = 90.0        # IMU無しのとき TURN_SPEED で何度/秒回るか(要実測)

# ---- 前進・センタリング ----
FORWARD_SPEED = 1500           # 前進の基本速さ
CENTER_KP = 2.0                # 左右距離を合わせる強さ(大きいほど強く曲がる)
CENTER_TURN_MAX = 500          # センタリング補正の上限(±)

CONTROL_HZ = 20                # 制御ループの周期(1秒あたりの判定回数)

# ---- LiDAR 内部 ----
POINT_LIFETIME = 0.20          # 点を保持する秒数(1回転ぶん強を確保)
DATA_TIMEOUT = 0.5             # この秒数 点が来なければ「LiDAR停止」とみなす

# 47バイトフレーム(MS200 / LD06 系)
FRAME_SIZE = 47
HEADER = 0x54
POINTS_PER_FRAME = 12


# ============================================================
# CRC8(MS200 / LD06 系)
# ============================================================
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


def _norm180(a):
    """角度を -180〜+180 に丸める。"""
    return (a + 180.0) % 360.0 - 180.0


def _angle_diff(a, b):
    """a - b を -180〜+180 で返す。"""
    d = a - b
    if d > 180:
        d -= 360
    if d < -180:
        d += 360
    return d


def quat_to_euler(qi, qj, qk, qr):
    """BNO085 のクォータニオンから (roll, pitch, yaw)[deg] を得る。"""
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


# ============================================================
# IMU(BNO085)で yaw を読み続ける。無くても動く(時間ベースに切替)。
# ============================================================
class YawTracker:

    def __init__(self):
        self._yaw = None
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self.ok = False

    def start(self):
        if not USE_IMU:
            print("IMU 未使用(時間でターンします)")
            return
        try:
            import board
            import busio
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
            i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
            self._bno = BNO08X_I2C(i2c)
            self._bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            self.ok = True
        except Exception as e:
            print("IMU 無効(時間でターンします):", e)
            self.ok = False
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("IMU started")

    def _loop(self):
        while self._running:
            try:
                qi, qj, qk, qr = self._bno.quaternion
                _, _, yaw = quat_to_euler(qi, qj, qk, qr)
                with self._lock:
                    self._yaw = _norm180(yaw)
            except Exception:
                pass
            time.sleep(0.005)

    def get_yaw(self):
        with self._lock:
            return self._yaw

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)


# ============================================================
# 旋回の進み具合(目標角度まで回ったか)を管理する
# ============================================================
class TurnState:

    def __init__(self):
        self.dir = None        # 'L' or 'R'
        self.target = 0.0      # 目標角度[deg]
        self.turned = 0.0      # 回った角度の累計[deg]
        self.prev_yaw = None
        self.t0 = 0.0
        self.use_yaw = False

    def begin(self, direction, target_deg, tracker):
        self.dir = direction
        self.target = target_deg
        self.turned = 0.0
        self.t0 = time.monotonic()
        self.use_yaw = bool(tracker and tracker.ok and tracker.get_yaw() is not None)
        self.prev_yaw = tracker.get_yaw() if self.use_yaw else None

    def update_done(self, tracker):
        """1ステップぶん進めて、目標角度に達したら True。"""
        if self.use_yaw:
            cur = tracker.get_yaw()
            if cur is not None and self.prev_yaw is not None:
                self.turned += abs(_angle_diff(cur, self.prev_yaw))
                self.prev_yaw = cur
        else:
            self.turned = (time.monotonic() - self.t0) * TURN_DEG_PER_SEC
        return self.turned >= self.target


# ============================================================
# LiDAR スキャナ(別スレッドで読み続ける)
# ============================================================
class LidarScanner:
    """点は (角度[deg: 正面0 / 右+ / 左-], 距離[mm], 受信時刻) で持つ。"""

    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self._points = deque()
        self._lock = threading.Lock()
        self._last_update = 0.0
        self._running = False
        self._thread = None

    def start(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
        self.ser.write(b"\xA5\xF5\xA2\xC1\x01\x81\xB3\x31\xF2")  # スキャン開始
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        print("LiDAR scanner started:", self.port)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser:
            self.ser.close()
        print("LiDAR scanner stopped")

    def _reader_loop(self):
        buf = bytearray()
        while self._running:
            try:
                data = self.ser.read(4096)
            except Exception as e:
                print("LiDAR serial error:", e)
                time.sleep(0.1)
                continue
            if data:
                buf.extend(data)
                self._parse(buf)

    def _parse(self, buf):
        new_points = []
        now = time.monotonic()
        while True:
            idx = buf.find(bytes([HEADER]))
            if idx < 0:
                buf.clear()
                break
            if len(buf) - idx < FRAME_SIZE:
                del buf[:idx]
                break
            frame = bytes(buf[idx:idx + FRAME_SIZE])
            if crc8(frame[:-1]) == frame[-1]:
                new_points.extend(self._decode(frame, now))
                del buf[:idx + FRAME_SIZE]
            else:
                del buf[:idx + 1]
        if new_points:
            with self._lock:
                self._points.extend(new_points)
                cutoff = now - POINT_LIFETIME
                while self._points and self._points[0][2] < cutoff:
                    self._points.popleft()
                self._last_update = now

    def _decode(self, frame, now):
        start = (frame[4] | (frame[5] << 8)) / 100.0
        end = (frame[42] | (frame[43] << 8)) / 100.0
        if end < start:
            end += 360.0
        span = end - start
        pts = []
        for k in range(POINTS_PER_FRAME):
            off = 6 + k * 3
            dist = frame[off] | (frame[off + 1] << 8)
            if dist <= 0:
                continue
            raw_angle = start + span * k / (POINTS_PER_FRAME - 1)
            a = _norm180(raw_angle - FRONT_OFFSET_DEG)
            if MIRROR:
                a = -a
            pts.append((a, float(dist), now))
        return pts

    # --- 判定 ---
    def has_data(self):
        with self._lock:
            fresh = (time.monotonic() - self._last_update) < DATA_TIMEOUT
            return fresh and len(self._points) > 0

    def _nearest(self, lo, hi):
        """角度 lo〜hi[deg] にある点の最短距離。無ければ無限大。"""
        best = float("inf")
        with self._lock:
            for a, d, _ in self._points:
                if lo <= a <= hi and d < best:
                    best = d
        return best

    # 真横の扇形(センタリングで距離そのものを使う)
    def left_distance(self):
        return self._nearest(-(SIDE_CENTER_DEG + SIDE_HALF_DEG),
                             -(SIDE_CENTER_DEG - SIDE_HALF_DEG))

    def right_distance(self):
        return self._nearest(SIDE_CENTER_DEG - SIDE_HALF_DEG,
                             SIDE_CENTER_DEG + SIDE_HALF_DEG)

    # 壁あり判定(15cm 以内か)
    def front_wall(self):
        return self._nearest(-FRONT_HALF_DEG, FRONT_HALF_DEG) < WALL_DIST_MM

    def left_wall(self):
        return self.left_distance() < WALL_DIST_MM

    def right_wall(self):
        return self.right_distance() < WALL_DIST_MM


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# ============================================================
# 走行(制御ループ)
# ============================================================
def main():
    scanner = LidarScanner()
    scanner.start()

    yaw = YawTracker()
    yaw.start()

    r = Robot()
    turn = TurnState()
    state = {"mode": "DRIVE"}   # "DRIVE" か "TURN"

    # --- 停止信号の対策(捕まえられる範囲だけ) ---
    # Ctrl+C(SIGINT)/ kill(SIGTERM)を KeyboardInterrupt に変えて、
    # r.loop() の後始末(速度0+クローズ)を走らせる。
    # ※ VSCode の停止ボタン(■)は SIGKILL のことが多く、これは捕まえ
    #    られない。確実に止めるなら Ctrl+C か電源スイッチ。
    def _graceful_stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _graceful_stop)
    signal.signal(signal.SIGTERM, _graceful_stop)

    def decide_turn():
        """前が壁のとき、回る向きと目標角度を決める。"""
        L = scanner.left_wall()
        R = scanner.right_wall()
        if not L and not R:
            d, deg, tag = "L", TURN90_DEG, "前のみ壁(左優先)"
        elif not R:
            d, deg, tag = "R", TURN90_DEG, "前+左が壁->右折"
        elif not L:
            d, deg, tag = "L", TURN90_DEG, "前+右が壁->左折"
        else:
            d, deg, tag = "L", TURN180_DEG, "行き止まり->180"
        print(f"[TURN] {tag}  L={int(L)} R={int(R)} dir={d} {deg:.0f}deg")
        return d, deg

    def step():
        if not scanner.has_data():
            r.stop()
            return

        # === 旋回中:目標角度まで他の判断をしない ===
        if state["mode"] == "TURN":
            done = turn.update_done(yaw)
            if turn.dir == "R":
                r.right(TURN_SPEED)
            else:
                r.left(TURN_SPEED)
            if done:
                state["mode"] = "DRIVE"
            return

        # === 走行中 ===
        if scanner.front_wall():
            # 旋回を開始(ここから目標角度まで TURN モード)
            d, deg = decide_turn()
            turn.begin(d, deg, yaw)
            state["mode"] = "TURN"
            if d == "R":
                r.right(TURN_SPEED)
            else:
                r.left(TURN_SPEED)
            return

        # 前進:左右の壁までの距離が等しくなるよう左右モーター量を調整
        ld = scanner.left_distance()
        rd = scanner.right_distance()
        if ld != float("inf") and rd != float("inf"):
            # 右が遠い(右に余裕)なら右へ寄せる -> turn 正=右。
            corr = _clamp(CENTER_KP * (rd - ld),
                          -CENTER_TURN_MAX, CENTER_TURN_MAX)
        else:
            corr = 0   # 片側しか壁が見えないときは真っ直ぐ
        r.drive(forward=FORWARD_SPEED, turn=corr)

    try:
        r.loop(step, hz=CONTROL_HZ)   # Ctrl+C / step=False で終了。自動で停止+クローズ。
    finally:
        scanner.stop()
        yaw.stop()


if __name__ == "__main__":
    main()