#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
all_sensor_dashboard.py
------------------------------------------------------------------
Raspberry Pi 5 用 全センサー統合テスト & ビジュアルダッシュボード

対応センサー
  - LSM6DSV16X     : IMU (Roll / Pitch / Yaw, 加速度, ジャイロ)
                     TCA9548A CH0 経由、FIFOバースト読み出し+相補フィルタ
                     (imu2.py のドライバをそのまま移植)
  - TCA9548A       : I2Cマルチプレクサ
      CH0          : LSM6DSV16X (IMU)
      CH1, CH2     : TCS34725 (RGBカラーセンサー) x2
      CH3, CH4     : VL53L1X  (ToF距離センサー) x2
  - ST7789         : 240x240 SPI LCD (表示先)

特徴
  - センサーごとに独立したスレッドでデータを取得
  - 画面は複数の「ページ」に分かれており、自動で切り替わる
    (ダッシュボード / IMU / カラー / 距離 / GAME)
  - ラズパイに直接つないだキーボード(またはSSH端末)の ← → キー
    (A/D, H/Lでも可) で手動ページ送りが可能。手動操作後は
    MANUAL_HOLD_SEC 秒だけ自動巡回が一時停止する
  - GPIO 4方向ボタン(button.py準拠, GPIO17/18/27/22 = 右/上/下/左)、
    evdev対応ゲームコントローラにも対応。
    右/左ボタンは通常ページ送り、上/下ボタンはGAMEページ専用
    (詳細は button_thread() のコメント参照)
  - GAMEページ: 4方向ボタンで遊べるNokia風スネークゲームを内蔵
  - 接続されていない・初期化に失敗したセンサーがあっても
    プログラム全体は落とさず、該当ページに "NO DATA" を表示する
  - CONFIG セクションのフラグで有効/無効を柔軟に切り替え可能
  - Ctrl+C で終了すると LCD の電源をオフにしてから終了する

必要ライブラリ (Raspberry Pi 5 / Raspberry Pi OS Bookworm 想定)
  pip install adafruit-circuitpython-vl53l1x \
              smbus2 pillow st7789 gpiozero evdev
------------------------------------------------------------------
"""

import sys
import time
import math
import copy
import random
import signal
import threading
import select

try:
    import tty
    import termios
    TTY_LIB_OK = True
except ImportError:
    # Windows等、termios非対応環境向けのフォールバック
    TTY_LIB_OK = False

from PIL import Image, ImageDraw, ImageFont

# ==================================================================
# CONFIG : 有効/無効フラグと各種パラメータ
#          (配線されていないセンサーは False にすれば安全にスキップされる)
# ==================================================================

ENABLE_IMU = True              # LSM6DSV16X (IMU, TCA9548A CH0)
ENABLE_TCA_COLOR = True        # TCA9548A CH1/CH2 : TCS34725 カラーセンサー
ENABLE_TCA_DISTANCE = True     # TCA9548A CH3/CH4 : VL53L1X 距離センサー
ENABLE_BUTTON = True           # GPIO 4方向ボタン(button.py準拠)での手動ページ切替 & GAME操作
ENABLE_KEYBOARD = True         # ラズパイ側キーボードの ← → キーでページ切り替え
ENABLE_CONTROLLER = True       # evdev対応ゲームコントローラでの操作 (スティック/ボタン)
ENABLE_GAME = True             # GAMEページ(Nokei風スネークゲーム)

AUTO_ROTATE = False             # ページを自動で巡回させるか (手動操作後は一定時間停止する)
PAGE_DURATION_SEC = 4.0        # 自動巡回時の1ページあたりの表示秒数
MANUAL_HOLD_SEC = 6.0          # 手動でページ切替した後、自動巡回を再開するまでの待機秒数
DISPLAY_FPS = 15               # 画面更新のフレームレート上限

# ---------- I2C アドレス ----------
TCA_ADDR = 0x70
TCS_ADDR = 0x29     # TCS34725 (RGBカラーセンサー)
VL_ADDR = 0x29       # VL53L1X  (別チャンネルなのでアドレス重複OK)
LSM6DSV16X_ADDR = 0x6B   # IMU (SDO/SA0=3.3V時。GND時は0x6A)

# ---------- TCA9548A チャンネル割り当て ----------
# IMU=CH0, カラーセンサー=CH1/CH2, 距離センサー=CH3/CH4 (実配線に合わせた割り当て)
TCA_CH_IMU = 0
TCA_CH_COLOR0 = 1
TCA_CH_COLOR1 = 2
TCA_CH_DIST0 = 3
TCA_CH_DIST1 = 4

# ---------- GPIO 4方向ボタン (button.py 準拠) ----------
# 表示上の呼称: "1"=右, "2"=上, "3"=下, "4"=左
BTN_RIGHT_PIN = 17   # "1" = 右
BTN_UP_PIN    = 18   # "2" = 上
BTN_DOWN_PIN  = 27   # "3" = 下
BTN_LEFT_PIN  = 22   # "4" = 左
BUTTON_BOUNCE_SEC = 0.05   # 機械的チャタリング対策 (gpiozero Button の bounce_time)

# ---------- LCD (ST7789) ----------
LCD_PORT = 0
LCD_CS = 0
LCD_DC = 25
LCD_RST = 24
LCD_W = 240
LCD_H = 240
LCD_SPI_HZ = 20_000_000

# ---------- GAME (Nokia風スネーク) ----------
GAME_CELL_PX = 10                 # 1マスのピクセルサイズ
GAME_GRID_W = LCD_W // GAME_CELL_PX          # 24マス
GAME_AREA_TOP = 26                # ヘッダー(26px)の下からプレイエリア開始
GAME_AREA_BOTTOM = LCD_H - 14     # フッターのページドット(14px)の上まで
GAME_GRID_H = (GAME_AREA_BOTTOM - GAME_AREA_TOP) // GAME_CELL_PX  # 20マス
GAME_TICK_SEC_START = 0.15        # 初期の1手あたりの秒数(遅い)
GAME_TICK_SEC_MIN = 0.07          # スコアが伸びた時の最速値
GAME_TICK_SPEEDUP_PER_FOOD = 0.004  # 餌を1個食べるごとに詰める秒数

# ---------- ゲームコントローラ (evdev) ----------
CONTROLLER_DEVICE = None          # 例: "/dev/input/event3" (Noneなら自動検出を試みる)
CONTROLLER_DEADZONE = 6000         # スティックの遊び(不感帯)。絶対値がこれ未満は無視
CONTROLLER_AXIS_MAX = 32767        # 一般的なコントローラの軸最大値 (機種によって異なる場合あり)
STICK_PAGE_THRESHOLD = 22000       # 左スティックでページ送りする際のしきい値(倒しきり判定)

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
    import smbus2
    from smbus2 import SMBus
    SMBUS_LIB_OK = True
except ImportError:
    SMBUS_LIB_OK = False

try:
    import adafruit_vl53l1x
    VL53_LIB_OK = True
except ImportError:
    VL53_LIB_OK = False

try:
    import ST7789
    ST7789_LIB_OK = True
except ImportError:
    ST7789_LIB_OK = False

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
# LSM6DSV16Xドライバ(imu2.py移植)はsmbus2のみに依存する(BNO085用のboard/busioは不要)
USE_IMU = ENABLE_IMU and SMBUS_LIB_OK
USE_TCA_COLOR = ENABLE_TCA_COLOR and SMBUS_LIB_OK
USE_TCA_DISTANCE = ENABLE_TCA_DISTANCE and VL53_LIB_OK and HW_I2C_LIB_OK and SMBUS_LIB_OK
USE_BUTTON = ENABLE_BUTTON and GPIOZERO_LIB_OK
USE_KEYBOARD = ENABLE_KEYBOARD and TTY_LIB_OK
USE_CONTROLLER = ENABLE_CONTROLLER and EVDEV_LIB_OK
USE_GAME = ENABLE_GAME


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


# 実行中に変更可能な設定値 (自動巡回ON/OFFなど)
settings_lock = threading.Lock()
runtime_settings = {
    "auto_rotate": AUTO_ROTATE,       # 自動ページ巡回 ON/OFF
}


def toggle_auto_rotate():
    with settings_lock:
        runtime_settings["auto_rotate"] = not runtime_settings["auto_rotate"]
        state = "ON" if runtime_settings["auto_rotate"] else "OFF"
    print(f"[PAD] 自動ページ巡回: {state}")


def set_state(key, **kwargs):
    """sensor_state の該当キーをロックしながら更新する共通ヘルパー"""
    with state_lock:
        sensor_state[key].update(kwargs)


def get_state_snapshot():
    """描画用にsensor_stateのコピーを取得する"""
    with state_lock:
        return copy.deepcopy(sensor_state)


# ==================================================================
# IMU (LSM6DSV16X) ドライバ本体  ※ imu2.py のドライバをそのまま移植
#   - TCA9548A CH0 の配下にあるため、reset/configure/FIFO読み出しの
#     いずれの前にも tca_select_channel(bus, TCA_CH_IMU) を発行し、
#     さらに他スレッド(カラー/距離)とチャンネル状態を取り合わないよう
#     i2c_lock で全体を保護する。
# ==================================================================

IMU_REG_FIFO_CTRL2    = 0x08
IMU_REG_FIFO_CTRL3    = 0x09
IMU_REG_FIFO_CTRL4    = 0x0A
IMU_REG_WHO_AM_I      = 0x0F
IMU_REG_CTRL1         = 0x10   # 加速度 ODR / モード
IMU_REG_CTRL2         = 0x11   # ジャイロ ODR / モード
IMU_REG_CTRL3         = 0x12   # BDU, IF_INC, SW_RESET
IMU_REG_CTRL6         = 0x15   # ジャイロ フルスケール, LPF1帯域
IMU_REG_CTRL8         = 0x17   # 加速度 フルスケール
IMU_REG_FIFO_STATUS1  = 0x1B
IMU_REG_FIFO_STATUS2  = 0x1C
IMU_REG_FIFO_DATA_OUT_TAG = 0x78

IMU_WHO_AM_I_VALUE = 0x70

IMU_TAG_GYRO_NC  = 0x01
IMU_TAG_ACCEL_NC = 0x02

IMU_ODR_CODE_1920HZ = 0b1010
IMU_GYRO_FS_2000DPS = 0b0100
IMU_ACCEL_FS_8G     = 0b10

IMU_GYRO_SENSITIVITY_MDPS_PER_LSB = 70.0     # ±2000dps時
IMU_ACCEL_SENSITIVITY_MG_PER_LSB  = 0.244    # ±8g時

IMU_FRAME_SIZE = 7
IMU_MAX_FRAMES_PER_TRANSFER = 8


class LSM6DSV16X:
    """imu2.py のドライバクラスをそのまま移植したもの。
    自身ではTCAチャンネル選択を行わない(呼び出し側がimu_thread内で
    i2c_lock保護のもとtca_select_channel()してから各メソッドを呼ぶ)。"""

    def __init__(self, bus, addr=LSM6DSV16X_ADDR):
        self.bus = bus
        self.addr = addr

    def write_reg(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def read_reg(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def read_regs(self, reg, length):
        write = smbus2.i2c_msg.write(self.addr, [reg])
        read = smbus2.i2c_msg.read(self.addr, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)

    def reset(self):
        self.write_reg(IMU_REG_CTRL3, 0x01)
        t0 = time.monotonic()
        while self.read_reg(IMU_REG_CTRL3) & 0x01:
            if time.monotonic() - t0 > 0.1:
                raise TimeoutError("LSM6DSV16X: SW_RESETが完了しない")
            time.sleep(0.001)

    def who_am_i(self):
        return self.read_reg(IMU_REG_WHO_AM_I)

    def configure(self):
        who = self.who_am_i()
        if who != IMU_WHO_AM_I_VALUE:
            raise RuntimeError(
                f"WHO_AM_I不一致: 0x{who:02X} (期待値 0x{IMU_WHO_AM_I_VALUE:02X})。"
                f"配線/アドレス(0x{self.addr:02X})/TCAチャンネルを確認してください。"
            )

        self.write_reg(IMU_REG_CTRL3, 0b01000100)                      # BDU=1, IF_INC=1
        self.write_reg(IMU_REG_CTRL1, (0b000 << 4) | IMU_ODR_CODE_1920HZ)  # 加速度 1.92kHz
        self.write_reg(IMU_REG_CTRL2, (0b000 << 4) | IMU_ODR_CODE_1920HZ)  # ジャイロ 1.92kHz
        self.write_reg(IMU_REG_CTRL6, (0b000 << 4) | IMU_GYRO_FS_2000DPS)  # LPF1最広帯域, ±2000dps
        self.write_reg(IMU_REG_CTRL8, IMU_ACCEL_FS_8G)                     # ±8g

        self.write_reg(IMU_REG_FIFO_CTRL3,
                        (IMU_ODR_CODE_1920HZ << 4) | IMU_ODR_CODE_1920HZ)  # BDR_GY/BDR_XL=1.92kHz
        self.write_reg(IMU_REG_FIFO_CTRL2, 0x00)

        self.write_reg(IMU_REG_FIFO_CTRL4, 0b000)   # bypassでクリア
        time.sleep(0.001)
        self.write_reg(IMU_REG_FIFO_CTRL4, 0b110)   # continuous

    def fifo_unread_count(self):
        status = self.read_regs(IMU_REG_FIFO_STATUS1, 2)
        diff_low = status[0]
        diff_high = status[1] & 0x01
        return diff_low | (diff_high << 8)

    def burst_read_fifo(self, n_frames):
        n_bytes = n_frames * IMU_FRAME_SIZE
        write = smbus2.i2c_msg.write(self.addr, [IMU_REG_FIFO_DATA_OUT_TAG])
        read = smbus2.i2c_msg.read(self.addr, n_bytes)
        self.bus.i2c_rdwr(write, read)
        return bytes(read)


def imu_to_int16(low, high):
    val = (high << 8) | low
    if val & 0x8000:
        val -= 0x10000
    return val


class ComplementaryFilter:
    """imu2.py と同じ相補フィルタ。dtはODR固定(1/1920s)から算出する。"""
    ALPHA = 0.98

    def __init__(self, odr_hz):
        self.dt = 1.0 / odr_hz
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self._last_accel_g = (0.0, 0.0, 1.0)

    def update_accel(self, ax_g, ay_g, az_g):
        self._last_accel_g = (ax_g, ay_g, az_g)

    def update_gyro(self, gx_dps, gy_dps, gz_dps):
        ax, ay, az = self._last_accel_g
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

        self.roll = self.ALPHA * (self.roll + gx_dps * self.dt) + (1.0 - self.ALPHA) * roll_acc
        self.pitch = self.ALPHA * (self.pitch + gy_dps * self.dt) + (1.0 - self.ALPHA) * pitch_acc
        self.yaw = (self.yaw + gz_dps * self.dt) % 360.0

    def angles(self):
        return self.roll, self.pitch, self.yaw


def imu_parse_fifo_frame(raw, off):
    tag = raw[off] >> 3
    x = imu_to_int16(raw[off + 1], raw[off + 2])
    y = imu_to_int16(raw[off + 3], raw[off + 4])
    z = imu_to_int16(raw[off + 5], raw[off + 6])
    return tag, x, y, z


def imu_update_from_frame(cf, tag, x, y, z):
    if tag == IMU_TAG_ACCEL_NC:
        cf.update_accel(
            x * IMU_ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0,
            y * IMU_ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0,
            z * IMU_ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0,
        )
    elif tag == IMU_TAG_GYRO_NC:
        cf.update_gyro(
            x * IMU_GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0,
            y * IMU_GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0,
            z * IMU_GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0,
        )
        return True
    return False


def imu_thread():
    """
    LSM6DSV16X (TCA9548A CH0) のFIFOをポーリングし、相補フィルタで
    roll/pitch/yawを更新してsensor_state["imu"]へ反映する。

    TCA9548Aは他スレッド(カラー/距離)と共有しているため、
    fifo_unread_count()/burst_read_fifo()を呼ぶ直前には毎回
    「i2c_lockを取ってからCH0を選択」をやり直す
    (他スレッドが直前にチャンネルを切り替えている可能性があるため)。
    このmux共有の都合上、センサー単体よりは実効サンプルレートが
    下がる点に注意(このドライバ単体の高速化については別途相談)。
    state_lock更新はstate_lockの競合を避けるため約50Hzに間引く。
    """
    if not USE_IMU:
        return

    bus = smbus2.SMBus(1)
    imu = LSM6DSV16X(bus)

    try:
        with i2c_lock:
            tca_select_channel(bus, TCA_CH_IMU)
            imu.reset()
        with i2c_lock:
            tca_select_channel(bus, TCA_CH_IMU)
            imu.configure()
        with i2c_lock:
            tca_select_channel(bus, TCA_CH_IMU)
            who = imu.who_am_i()
        print(f"[IMU] LSM6DSV16X 初期化完了 (WHO_AM_I=0x{who:02X})")
    except Exception as e:
        print("[IMU] 初期化に失敗しました:", e)
        set_state("imu", ok=False)
        return

    cf = ComplementaryFilter(odr_hz=1920)

    sample_count = 0
    t_report = time.monotonic()
    last_state_push = 0.0
    STATE_PUSH_INTERVAL = 1.0 / 50.0   # state_lockの競合を避けるための更新間引き

    while running:
        try:
            with i2c_lock:
                tca_select_channel(bus, TCA_CH_IMU)
                n = imu.fifo_unread_count()

            if n < 2:
                time.sleep(0.0005)
                continue

            n_read = min(n, IMU_MAX_FRAMES_PER_TRANSFER)
            with i2c_lock:
                tca_select_channel(bus, TCA_CH_IMU)
                raw = imu.burst_read_fifo(n_read)

            for i in range(n_read):
                off = i * IMU_FRAME_SIZE
                tag, x, y, z = imu_parse_fifo_frame(raw, off)
                updated = imu_update_from_frame(cf, tag, x, y, z)

                if updated:
                    sample_count += 1
                    now = time.monotonic()
                    if now - last_state_push >= STATE_PUSH_INTERVAL:
                        roll, pitch, yaw = cf.angles()
                        ax, ay, az = cf._last_accel_g
                        rate = sample_count / (now - t_report) if now > t_report else 0.0
                        set_state(
                            "imu", ok=True,
                            roll=roll, pitch=pitch, yaw=yaw,
                            ax=ax, ay=ay, az=az,
                            hz=rate,
                        )
                        sample_count = 0
                        t_report = now
                        last_state_push = now

        except Exception as e:
            print("[IMU] 読み取りエラー:", e)
            set_state("imu", ok=False)
            time.sleep(0.5)


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
            tca_select_channel(bus, TCA_CH_COLOR0)
            tcs_init(bus)
            tca_select_channel(bus, TCA_CH_COLOR1)
            tcs_init(bus)
    except Exception as e:
        print("[COLOR] 初期化に失敗しました:", e)
        set_state("color0", ok=False)
        set_state("color1", ok=False)
        return

    while running:
        for ch, key in ((TCA_CH_COLOR0, "color0"), (TCA_CH_COLOR1, "color1")):
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
            tca_select_channel(bus, TCA_CH_DIST0)
            vl_ch2 = adafruit_vl53l1x.VL53L1X(i2c)
            vl_ch2.start_ranging()

            tca_select_channel(bus, TCA_CH_DIST1)
            vl_ch3 = adafruit_vl53l1x.VL53L1X(i2c)
            vl_ch3.start_ranging()

    except Exception as e:
        print("[DIST] 初期化に失敗しました:", e)
        set_state("dist2", ok=False)
        set_state("dist3", ok=False)
        return

    sensors = ((TCA_CH_DIST0, "dist2", vl_ch2), (TCA_CH_DIST1, "dist3", vl_ch3))

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
# GAME : Nokia風スネークゲーム
# ==================================================================

GAME_DIR_UP    = (0, -1)
GAME_DIR_DOWN  = (0, 1)
GAME_DIR_LEFT  = (-1, 0)
GAME_DIR_RIGHT = (1, 0)

GAME_STATE_READY     = "READY"
GAME_STATE_PLAYING   = "PLAYING"
GAME_STATE_GAME_OVER = "GAME_OVER"

GAME_PAGE_TITLE = "GAME"


class SnakeGame:
    """
    Nokia風スネークゲーム。座標はセル単位 (0,0)〜(GAME_GRID_W-1,GAME_GRID_H-1)。

    状態遷移: READY(開始待ち) -> PLAYING(プレイ中) -> GAME_OVER(結果表示)
              GAME_OVER/READY で上ボタン -> 新しいゲームでPLAYINGへ

    ルール:
      - 進行方向の真逆への転換は無視する(即死防止の定番ルール)
      - 壁に当たる、または自分の胴体に当たるとGAME_OVER
        (Nokia Snake同様、壁でのラップアラウンドはしない)
      - 餌を食べるごとにスコア+1、1手あたりの間隔(tick_sec)を少しずつ短縮

    set_direction()/start()はボタンのコールバックスレッドから、
    step()はgame_tick_threadから呼ばれるため、内部ロックで保護する。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.high_score = 0
        self._reset_locked()

    def _reset_locked(self):
        cx, cy = GAME_GRID_W // 2, GAME_GRID_H // 2
        self.snake = [(cx - 1, cy), (cx - 2, cy), (cx - 3, cy)]  # 先頭 = index0
        self.direction = GAME_DIR_RIGHT
        self.pending_direction = GAME_DIR_RIGHT
        self.score = 0
        self.tick_sec = GAME_TICK_SEC_START
        self.state = GAME_STATE_READY
        self.food = self._spawn_food_locked()

    def _spawn_food_locked(self):
        occupied = set(self.snake)
        while True:
            fx = random.randrange(GAME_GRID_W)
            fy = random.randrange(GAME_GRID_H)
            if (fx, fy) not in occupied:
                return (fx, fy)

    def start(self):
        """READY / GAME_OVER から新しいゲームを開始する。PLAYING中の呼び出しは無視。"""
        with self._lock:
            if self.state == GAME_STATE_PLAYING:
                return
            self._reset_locked()
            self.state = GAME_STATE_PLAYING

    def set_direction(self, d):
        with self._lock:
            if self.state != GAME_STATE_PLAYING:
                return
            if d[0] == -self.direction[0] and d[1] == -self.direction[1]:
                return  # 真逆は無視
            self.pending_direction = d

    def is_playing(self):
        with self._lock:
            return self.state == GAME_STATE_PLAYING

    def get_tick_sec(self):
        with self._lock:
            return self.tick_sec

    def step(self):
        """1手進める(game_tick_threadからtick_sec間隔で呼ばれる)"""
        with self._lock:
            if self.state != GAME_STATE_PLAYING:
                return

            self.direction = self.pending_direction
            hx, hy = self.snake[0]
            nx, ny = hx + self.direction[0], hy + self.direction[1]

            if not (0 <= nx < GAME_GRID_W and 0 <= ny < GAME_GRID_H):
                self.state = GAME_STATE_GAME_OVER
                self.high_score = max(self.high_score, self.score)
                return

            ate = (nx, ny) == self.food
            # 食べない時は尻尾マスがこの後動くので、自己衝突判定から除外してよい
            body_check = self.snake if ate else self.snake[:-1]
            if (nx, ny) in body_check:
                self.state = GAME_STATE_GAME_OVER
                self.high_score = max(self.high_score, self.score)
                return

            self.snake.insert(0, (nx, ny))
            if ate:
                self.score += 1
                self.tick_sec = max(GAME_TICK_SEC_MIN,
                                     self.tick_sec - GAME_TICK_SPEEDUP_PER_FOOD)
                self.food = self._spawn_food_locked()
            else:
                self.snake.pop()

    def snapshot(self):
        with self._lock:
            return {
                "state": self.state,
                "snake": list(self.snake),
                "food": self.food,
                "score": self.score,
                "high_score": self.high_score,
            }


snake_game = SnakeGame()


def game_tick_thread():
    """PLAYING中のみ一定間隔でsnake_game.step()を呼ぶ。停止中はアイドルポーリング。"""
    if not USE_GAME:
        return
    while running:
        if snake_game.is_playing():
            snake_game.step()
            time.sleep(snake_game.get_tick_sec())
        else:
            time.sleep(0.05)


def is_game_page_active():
    with page_lock:
        idx = current_page_idx
    if 0 <= idx < len(PAGE_DEFS):
        return PAGE_DEFS[idx][0] == GAME_PAGE_TITLE
    return False


# ==================================================================
# GPIO 4方向ボタンによる手動ページ切り替え & GAME操作 (button.py 準拠)
# ==================================================================

def _button_pressed_log(n, label):
    """button.py の pressed(n) を踏襲したログ出力"""
    print(f"Button {n} pressed ({label})")


def button_thread(num_pages):
    """
    GPIO 4方向ボタン (button.py 準拠: GPIO17/18/27/22)。
    表示上の呼称は "1"=右, "2"=上, "3"=下, "4"=左。

    役割分担:
      - 上(2)/下(3) : GAMEページでのみ意味を持つ。他のページでは完全に無視する。
                       GAMEページでは常にスネークの移動方向として使う。
                       上ボタンはREADY/GAME_OVER中には「開始/再挑戦」を兼ねる。
      - 右(1)/左(4) : 通常はchange_page()でページ送り。
                       ただし「GAMEページを表示中 かつ PLAYING中」だけは
                       スネークの左右移動として奪われる
                       (4方向操作のスネークにはどうしても右/左も必要なため)。
                       READY/GAME_OVER中はGAMEページにいてもページ送りとして働くので、
                       操作を奪われたままページを抜けられなくなることはない。
    """
    if not USE_BUTTON:
        return

    try:
        btn_right = Button(BTN_RIGHT_PIN, pull_up=True, bounce_time=BUTTON_BOUNCE_SEC)
        btn_up    = Button(BTN_UP_PIN,    pull_up=True, bounce_time=BUTTON_BOUNCE_SEC)
        btn_down  = Button(BTN_DOWN_PIN,  pull_up=True, bounce_time=BUTTON_BOUNCE_SEC)
        btn_left  = Button(BTN_LEFT_PIN,  pull_up=True, bounce_time=BUTTON_BOUNCE_SEC)
    except Exception as e:
        print("[BUTTON] 初期化に失敗しました:", e)
        return

    print(f"[BUTTON] 1(右)=GPIO{BTN_RIGHT_PIN} 2(上)=GPIO{BTN_UP_PIN} "
          f"3(下)=GPIO{BTN_DOWN_PIN} 4(左)=GPIO{BTN_LEFT_PIN}")

    def game_playing_and_active():
        return USE_GAME and is_game_page_active() and snake_game.is_playing()

    def on_up():
        _button_pressed_log(2, "上")
        if not (USE_GAME and is_game_page_active()):
            return
        if snake_game.is_playing():
            snake_game.set_direction(GAME_DIR_UP)
        else:
            snake_game.start()  # READY/GAME_OVERから上ボタンで開始・再挑戦

    def on_down():
        _button_pressed_log(3, "下")
        if game_playing_and_active():
            snake_game.set_direction(GAME_DIR_DOWN)

    def on_right():
        _button_pressed_log(1, "右")
        if game_playing_and_active():
            snake_game.set_direction(GAME_DIR_RIGHT)
        else:
            change_page(+1, num_pages)

    def on_left():
        _button_pressed_log(4, "左")
        if game_playing_and_active():
            snake_game.set_direction(GAME_DIR_LEFT)
        else:
            change_page(-1, num_pages)

    btn_up.when_pressed = on_up
    btn_down.when_pressed = on_down
    btn_right.when_pressed = on_right
    btn_left.when_pressed = on_left

    # gpiozeroのButtonコールバックはライブラリ内部のスレッドで処理されるため、
    # このスレッドはrunningがFalseになるまで待機するだけでよい。
    while running:
        time.sleep(0.2)


# ==================================================================
# ラズパイ側キーボードによる手動ページ切り替え (← → キー)
# ==================================================================

def keyboard_thread(num_pages):
    """
    ラズパイに直接つないだキーボード(またはSSH端末)から
    矢印キー ← / → でページを手動切り替えする。
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
        "BTN_TR":    lambda: jump_to_page(GAME_PAGE_TITLE),    # R1相当: GAMEページへ直接ジャンプ
        "BTN_TL":    lambda: jump_to_page("DASHBOARD"),       # L1相当: ダッシュボードへ直接ジャンプ
        "BTN_START": toggle_auto_rotate,                       # STARTボタン : 自動巡回ON/OFF
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
        print("[PAD] コントローラが見つかりませんでした")
        return

    print(f"[PAD] 接続しました: {dev.name} ({dev.path})")
    print("[PAD] 左スティック / 十字キー = ゲーム操作 / ページ切替")

    def game_playing_and_active():
        return USE_GAME and is_game_page_active() and snake_game.is_playing()

    def on_up():
        _button_pressed_log(2, "上")
        if not (USE_GAME and is_game_page_active()):
            return
        if snake_game.is_playing():
            snake_game.set_direction(GAME_DIR_UP)
        else:
            snake_game.start()  # READY/GAME_OVERから上ボタンで開始・再挑戦

    def on_down():
        _button_pressed_log(3, "下")
        if game_playing_and_active():
            snake_game.set_direction(GAME_DIR_DOWN)

    def on_right():
        _button_pressed_log(1, "右")
        if game_playing_and_active():
            snake_game.set_direction(GAME_DIR_RIGHT)
        else:
            change_page(+1, num_pages)

    def on_left():
        _button_pressed_log(4, "左")
        if game_playing_and_active():
            snake_game.set_direction(GAME_DIR_LEFT)
        else:
            change_page(-1, num_pages)

    # Switch Pro Controller ボタン → ゲーム操作

    button_actions = {
        ecodes.BTN_EAST: on_right,         # A
        ecodes.BTN_Y: on_left,
        ecodes.BTN_X: on_up,
        ecodes.BTN_SOUTH:  on_down,       # B
        ecodes.BTN_TL:    lambda: jump_to_page("DASHBOARD"),  # L
        ecodes.BTN_TR:    lambda: jump_to_page(GAME_PAGE_TITLE), # R
        ecodes.BTN_START: toggle_auto_rotate,  # +
    }

    # 左スティック（Switch Pro は 0〜255）
    lx = 128
    ly = 128
    center = 128
    DEADZONE = 20
    THRESHOLD = 200

    try:
        for event in dev.read_loop():

            # -----------------------------
            # ボタン
            # -----------------------------
            if event.type == ecodes.EV_KEY:
                if event.value == 1:  # 押された瞬間のみ
                    action = button_actions.get(event.code)
                    if action:
                        action()
                    else:
                        print(f"[PAD] 未割当ボタン code={event.code}")

            # -----------------------------
            # アナログスティック
            # -----------------------------
            elif event.type == ecodes.EV_ABS:

                # 左右
                if event.code == ecodes.ABS_X:
                    lx = event.value

                # 上下
                elif event.code == ecodes.ABS_Y:
                    ly = event.value

                # 十字キー（Switch Pro）
                elif event.code == ecodes.ABS_HAT0X:
                    if event.value == 1:
                        on_right()
                    elif event.value == -1:
                        on_left()

                elif event.code == ecodes.ABS_HAT0Y:
                    if event.value == -1:
                        on_up()
                    elif event.value == 1:
                        on_down()

            # -----------------------------
            # 左スティック左右 → ページ切替 or ゲーム操作
            # -----------------------------
            if lx > THRESHOLD:
                on_right()
            elif lx < (255 - THRESHOLD):
                on_left()

            # -----------------------------
            # 左スティック上下 → ゲーム操作
            # -----------------------------
            if ly < (center - THRESHOLD):
                on_up()
            elif ly > (center + THRESHOLD):
                on_down()

    except KeyboardInterrupt:
        print("[PAD] 終了します")



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
    draw.rectangle([0, 0, LCD_W, 26], fill=(22, 24, 30))
    draw.text((10, 5), title, font=FONTS["title"], fill=COL_TEXT)

    dots = [
        ("I", state["imu"]["ok"]),
        ("C", state["color0"]["ok"] or state["color1"]["ok"]),
        ("D", state["dist2"]["ok"] or state["dist3"]["ok"]),
    ]
    dx = LCD_W - 12
    for label, ok in reversed(dots):
        draw_status_dot(draw, dx, 13, ok)
        dx -= 16


def draw_page_dots(draw, page_idx, num_pages):
    total_w = num_pages * 14
    start_x = (LCD_W - total_w) // 2 + 7
    y = LCD_H - 10
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
    cw = (LCD_W - pad * 3) // 2
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
        draw.text((x1 + 8, y0 + 54), "CH1 / CH2", font=FONTS["tiny"], fill=COL_SUBTEXT)
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

    # --- GAMEカード ---
    draw_card(draw, x1, y1, cw, ch, "GAME")
    if USE_GAME:
        g = snake_game.snapshot()
        draw.text((x1 + 8, y1 + 22), f"Score {g['score']}",
                   font=FONTS["value"], fill=COL_TEXT)
        draw.text((x1 + 8, y1 + 50), f"{g['state']}  Best {g['high_score']}",
                   font=FONTS["small"], fill=COL_SUBTEXT)
    else:
        draw.text((x1 + 8, y1 + 34), "NO DATA", font=FONTS["label"], fill=COL_ERROR)


def render_imu(draw, state):
    y0 = 34
    if not state["imu"]["ok"]:
        draw.text((LCD_W // 2, LCD_H // 2), "NO DATA", font=FONTS["value"],
                   fill=COL_ERROR, anchor="mm")
        return

    draw_attitude_indicator(draw, 62, y0 + 50, 46,
                             state["imu"]["roll"], state["imu"]["pitch"])
    draw_compass(draw, LCD_W - 62, y0 + 50, 46, state["imu"]["yaw"])

    draw.text((20, y0 + 100), f"R {state['imu']['roll']:+.1f}", font=FONTS["label"], fill=COL_TEXT)
    draw.text((100, y0 + 100), f"P {state['imu']['pitch']:+.1f}", font=FONTS["label"], fill=COL_TEXT)
    draw.text((180, y0 + 100), f"Y {state['imu']['yaw']:+.1f}", font=FONTS["label"], fill=COL_TEXT)

    # 加速度バー (LSM6DSV16Xは重力込みの生値, 単位g。表示レンジは±4gを満スケールとする)
    ay0 = y0 + 122
    accel = (("Ax", state["imu"]["ax"]), ("Ay", state["imu"]["ay"]), ("Az", state["imu"]["az"]))
    for i, (label, val) in enumerate(accel):
        yy = ay0 + i * 20
        draw.text((10, yy), label, font=FONTS["small"], fill=COL_SUBTEXT)
        ratio = max(0.0, min(1.0, (val + 4.0) / 8.0))
        draw_bar(draw, 40, yy + 2, 150, 10, ratio, COL_ACCENT)
        draw.text((195, yy), f"{val:+.2f}g", font=FONTS["tiny"], fill=COL_TEXT)


def render_color(draw, state):
    y0 = 34
    pad = 10
    box_w = (LCD_W - pad * 3) // 2
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
        x = pad + i * (LCD_W // 2)
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


COL_SNAKE_HEAD = (140, 255, 120)
COL_SNAKE_BODY = (70, 190, 90)
COL_FOOD = (255, 90, 90)
COL_GAME_BG = (10, 30, 14)   # Nokia液晶風の深緑背景


def render_game(draw, state):
    """
    GAMEページ: Nokia風スネークゲームの描画。
    ゲームの状態(READY/PLAYING/GAME_OVER)はsnake_game.snapshot()から
    毎フレーム取得する(他のページと同じ「スナップショットを描くだけ」の設計)。
    """
    if not USE_GAME:
        draw.text((LCD_W // 2, LCD_H // 2), "NO DATA", font=FONTS["value"],
                   fill=COL_ERROR, anchor="mm")
        return

    g = snake_game.snapshot()

    area_x0, area_y0 = 0, GAME_AREA_TOP
    area_w = GAME_GRID_W * GAME_CELL_PX
    area_h = GAME_GRID_H * GAME_CELL_PX
    draw.rectangle([area_x0, area_y0, area_x0 + area_w, area_y0 + area_h], fill=COL_GAME_BG)

    def cell_rect(cx, cy):
        x0 = area_x0 + cx * GAME_CELL_PX
        y0 = area_y0 + cy * GAME_CELL_PX
        return [x0 + 1, y0 + 1, x0 + GAME_CELL_PX - 1, y0 + GAME_CELL_PX - 1]

    # 餌
    fx, fy = g["food"]
    draw.rectangle(cell_rect(fx, fy), fill=COL_FOOD)

    # 蛇 (先頭だけ明るい色にする)
    for i, (sx, sy) in enumerate(g["snake"]):
        color = COL_SNAKE_HEAD if i == 0 else COL_SNAKE_BODY
        draw.rectangle(cell_rect(sx, sy), fill=color)

    # スコア表示
    draw.text((6, area_y0 + area_h + 2), f"Score {g['score']}  Best {g['high_score']}",
               font=FONTS["tiny"], fill=COL_SUBTEXT)

    if g["state"] == GAME_STATE_READY:
        draw.text((LCD_W // 2, area_y0 + area_h // 2 - 10), "SNAKE",
                   font=FONTS["value"], fill=COL_SNAKE_HEAD, anchor="mm")
        draw.text((LCD_W // 2, area_y0 + area_h // 2 + 16), "UP(2)で開始",
                   font=FONTS["small"], fill=COL_TEXT, anchor="mm")
    elif g["state"] == GAME_STATE_GAME_OVER:
        draw.text((LCD_W // 2, area_y0 + area_h // 2 - 10), "GAME OVER",
                   font=FONTS["value"], fill=COL_ERROR, anchor="mm")
        draw.text((LCD_W // 2, area_y0 + area_h // 2 + 16),
                   f"Score {g['score']}  UP(2)で再挑戦",
                   font=FONTS["small"], fill=COL_TEXT, anchor="mm")


PAGE_DEFS = [
    ("DASHBOARD", render_dashboard),
    ("IMU", render_imu),
    ("COLOR", render_color),
    ("DISTANCE", render_distance),
]
if USE_GAME:
    PAGE_DEFS.append((GAME_PAGE_TITLE, render_game))


# ==================================================================
# 描画スレッド (メインループ)
# ==================================================================

def init_display():
    if not ST7789_LIB_OK:
        print("[LCD] ST7789ライブラリが見つかりません。描画のみ確認用に画面出力はスキップします。")
        return None
    try:
        disp = ST7789.ST7789(
            port=LCD_PORT, cs=LCD_CS, dc=LCD_DC, rst=LCD_RST,
            width=LCD_W, height=LCD_H, rotation=90,
            spi_speed_hz=LCD_SPI_HZ,
        )
        disp.begin()
        disp._spi.mode = 3
        disp.reset()
        disp._init()
        return disp
    except Exception as e:
        print("[LCD] 初期化に失敗しました:", e)
        return None


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

        img = Image.new("RGB", (LCD_W, LCD_H), COL_BG)
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
                disp.display(img)
            except Exception as e:
                print("[LCD] 描画エラー:", e)

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
            disp.command(0x28)  # ディスプレイOFF
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
    print(f"   IMU(LSM6DSV16X)  : {'有効' if USE_IMU else '無効'}")
    print(f"   COLOR(TCS34725)  : {'有効' if USE_TCA_COLOR else '無効'}")
    print(f"   DISTANCE(VL53L1X): {'有効' if USE_TCA_DISTANCE else '無効'}")
    print(f"   BUTTON(4方向)    : {'有効' if USE_BUTTON else '無効'}")
    print(f"   KEYBOARD         : {'有効' if USE_KEYBOARD else '無効'}")
    print(f"   CONTROLLER(evdev): {'有効' if USE_CONTROLLER else '無効'}")
    print(f"   GAME(Snake)      : {'有効' if USE_GAME else '無効'}")
    print("======================================================")

    disp = init_display()

    threads = []
    threads.append(threading.Thread(target=imu_thread, daemon=True))
    threads.append(threading.Thread(target=tca_color_thread, daemon=True))
    threads.append(threading.Thread(target=tca_distance_thread, daemon=True))

    if USE_GAME:
        threads.append(threading.Thread(target=game_tick_thread, daemon=True))

    if USE_BUTTON:
        threads.append(threading.Thread(target=button_thread, args=(len(PAGE_DEFS),), daemon=True))

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