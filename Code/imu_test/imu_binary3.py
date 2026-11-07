#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LSM6DSV16X (秋月電子 6軸IMUセンサーモジュール, akizukidenshi.com/catalog/g/g130950/)
Raspberry Pi 5 / I2C1 (/dev/i2c-1) 用ドライバ

FIFOをContinuousモードで動作させ、ジャイロ+加速度を1.92kHzでバッチングし、
たまったフレームを1回のI2Cトランザクションでバースト読み出し、
相補フィルタでロール/ピッチ/ヨー角を高速に更新し続ける。

参照: LSM6DSV16Xデータシート (DS13510 Rev.4)
  - I2Cスレーブアドレス: 5.1.2節 Table12 (SAD=110101xb, SDO/SA0=GND時 0x6A)
  - レジスタマップ: 9章 Register mapping table
  - FIFO動作: 6.12節, FIFO_CTRL1-4 (07h-0Ah): 9.5-9.8節
  - FIFO_STATUS1/2 (1Bh/1Ch): 9.25-9.26節
  - FIFO_DATA_OUT_TAG (78h) 以降とTAGコード表: 9.84節 Table218
  - CTRL1/CTRL2 (ODR): 9.14-9.15節, CTRL6 (ジャイロFS): 9.19節, CTRL8 (加速度FS): 9.21節
  - 感度表 (mg/LSB, mdps/LSB): Table 6 (Mechanical characteristics)

必要パッケージ: pip3 install smbus2 --break-system-packages
I2C1を有効化: sudo raspi-config -> Interface Options -> I2C
確認: i2cdetect -y 1  (0x6a に見えるはず)
"""

import time
import math
import smbus2

# ------------------------------------------------------------------
# I2C設定
# ------------------------------------------------------------------
I2C_BUS = 1                # Raspberry Pi 5 の I2C1 (/dev/i2c-1)
I2C_ADDR = 0x6B            # SDO/SA0=GND (モジュールのデフォルト)。SDO=3.3Vなら0x6B

# ------------------------------------------------------------------
# レジスタアドレス (データシート 9章 Register mapping)
# ------------------------------------------------------------------
REG_FIFO_CTRL1        = 0x07
REG_FIFO_CTRL2        = 0x08
REG_FIFO_CTRL3        = 0x09
REG_FIFO_CTRL4        = 0x0A
REG_WHO_AM_I           = 0x0F
REG_CTRL1              = 0x10   # 加速度 ODR / モード
REG_CTRL2              = 0x11   # ジャイロ ODR / モード
REG_CTRL3              = 0x12   # BDU, IF_INC, SW_RESET
REG_CTRL6              = 0x15   # ジャイロ フルスケール, LPF1帯域
REG_CTRL8              = 0x17   # 加速度 フルスケール
REG_FIFO_STATUS1       = 0x1B
REG_FIFO_STATUS2       = 0x1C
REG_FIFO_DATA_OUT_TAG  = 0x78   # 以降 0x79-0x7E に X_L,X_H,Y_L,Y_H,Z_L,Z_H が続く(1フレーム7byte)

WHO_AM_I_VALUE = 0x70

# FIFO_DATA_OUT_TAG の TAG_SENSOR_[4:0] コード (Table 218)
TAG_GYRO_NC  = 0x01
TAG_ACCEL_NC = 0x02

# ------------------------------------------------------------------
# センサ設定値 (ODR=1.92kHz, ジャイロ±2000dps, 加速度±8g)
# ODR_XL/ODR_G = 1010b = 1.92kHz (Table 52 / Table 55)
# ------------------------------------------------------------------
ODR_CODE_1920HZ = 0b1010
GYRO_FS_2000DPS = 0b0100          # CTRL6 FS_G[3:0] (9.19節)
ACCEL_FS_8G     = 0b10            # CTRL8 FS_XL[1:0] (9.21節)

GYRO_SENSITIVITY_MDPS_PER_LSB  = 70.0     # ±2000dps時 (Mechanical characteristics表)
ACCEL_SENSITIVITY_MG_PER_LSB   = 0.244    # ±8g時

FRAME_SIZE = 7                     # TAG(1) + X_L/X_H/Y_L/Y_H/Z_L/Z_H(6)
MAX_FRAMES_PER_TRANSFER = 8       # 1回のI2Cトランザクションで読むフレーム数上限(安全マージン)


class LSM6DSV16X:
    def __init__(self, bus_num=I2C_BUS, addr=I2C_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr

    # ---- 基本I/O ----
    def write_reg(self, reg, value):
        self.bus.write_byte_data(self.addr, reg, value)

    def read_reg(self, reg):
        return self.bus.read_byte_data(self.addr, reg)

    def read_regs(self, reg, length):
        # smbus2のブロック読み出し(0x1C等の連続ステータス読み出し用, 32byte以下)
        write = smbus2.i2c_msg.write(self.addr, [reg])
        read = smbus2.i2c_msg.read(self.addr, length)
        self.bus.i2c_rdwr(write, read)
        return list(read)

    # ---- 初期化 ----
    def reset(self):
        # CTRL3(12h): SW_RESET=1 (bit0), BDU=1(bit6), IF_INC=1(bit2) は自動的に既定値へ戻る
        self.write_reg(REG_CTRL3, 0x01)
        # SW_RESETは自動クリアされる。クリアされるまで待つ(データシート: 数十us程度)
        t0 = time.monotonic()
        while self.read_reg(REG_CTRL3) & 0x01:
            if time.monotonic() - t0 > 0.1:
                raise TimeoutError("LSM6DSV16X: SW_RESETが完了しない")
            time.sleep(0.001)

    def who_am_i(self):
        return self.read_reg(REG_WHO_AM_I)

    def configure(self):
        who = self.who_am_i()
        if who != WHO_AM_I_VALUE:
            raise RuntimeError(
                f"WHO_AM_I不一致: 0x{who:02X} (期待値 0x{WHO_AM_I_VALUE:02X})。"
                f"配線/アドレス(0x{self.addr:02X})を確認してください。"
            )

        # CTRL3(12h): BDU=1, IF_INC=1 (連続アドレス読み出しを有効化 -> FIFOバースト読みに必須)
        self.write_reg(REG_CTRL3, 0b01000100)

        # CTRL1(10h): 加速度 OP_MODE=high-performance(000), ODR=1.92kHz
        self.write_reg(REG_CTRL1, (0b000 << 4) | ODR_CODE_1920HZ)

        # CTRL2(11h): ジャイロ OP_MODE=high-performance(000), ODR=1.92kHz
        self.write_reg(REG_CTRL2, (0b000 << 4) | ODR_CODE_1920HZ)

        # CTRL6(15h): ジャイロLPF1帯域=000(最広帯域, 低遅延優先), FS_G=±2000dps
        self.write_reg(REG_CTRL6, (0b000 << 4) | GYRO_FS_2000DPS)

        # CTRL8(17h): 加速度LPF2/HPF=デフォルト(バイパス, ODR/2), FS_XL=±8g
        self.write_reg(REG_CTRL8, ACCEL_FS_8G)

        # FIFO_CTRL3(09h): BDR_GY=1.92kHz(bit7:4), BDR_XL=1.92kHz(bit3:0)
        self.write_reg(REG_FIFO_CTRL3, (ODR_CODE_1920HZ << 4) | ODR_CODE_1920HZ)

        # FIFO_CTRL2(08h): 圧縮なし, STOP_ON_WTM無効(継続動作)
        self.write_reg(REG_FIFO_CTRL2, 0x00)

        # FIFOをいったんBypassにしてクリアしてからContinuousへ (データシート 6.12.2節の手順)
        self.write_reg(REG_FIFO_CTRL4, 0b000)          # FIFO_MODE=000 bypass
        time.sleep(0.001)
        self.write_reg(REG_FIFO_CTRL4, 0b110)           # FIFO_MODE=110 continuous

    # ---- FIFO読み出し ----
    def fifo_unread_count(self):
        # FIFO_STATUS1(1Bh)=DIFF_FIFO[7:0], FIFO_STATUS2(1Ch)のbit0=DIFF_FIFO_8
        status = self.read_regs(REG_FIFO_STATUS1, 2)
        diff_low = status[0]
        diff_high = status[1] & 0x01
        return diff_low | (diff_high << 8)

    def burst_read_fifo(self, n_frames):
        """
        FIFO_DATA_OUT_TAG(78h)から n_frames*7 byte を1トランザクションで読み出す。
        IF_INC=1のため0x78-0x7Eの窓を読み終えると自動的に0x78へ折り返し、
        次のFIFOワードが取得できる(ST社FIFO出力レジスタの標準仕様)。
        戻り値: bytesオブジェクト (長さ n_frames*7)
        """
        n_bytes = n_frames * FRAME_SIZE
        write = smbus2.i2c_msg.write(self.addr, [REG_FIFO_DATA_OUT_TAG])
        read = smbus2.i2c_msg.read(self.addr, n_bytes)
        self.bus.i2c_rdwr(write, read)
        return bytes(read)

    def close(self):
        self.bus.close()


def to_int16(low, high):
    val = (high << 8) | low
    if val & 0x8000:
        val -= 0x10000
    return val


class ComplementaryFilter:
    """
    加速度(重力方向)でロール/ピッチのドリフトを補正し、
    ヨーはジャイロ積分のみ(磁気センサが無いため絶対方位補正はできない)。
    dtはODR固定(1/1920s)から算出 -> ホスト側のタイマジッタの影響を受けない。
    """
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

def main():
    imu = LSM6DSV16X()
    imu.reset()
    imu.configure()
    print(f"LSM6DSV16X 初期化完了 (WHO_AM_I=0x{imu.who_am_i():02X})")

    cf = ComplementaryFilter(odr_hz=1920)

    sample_count = 0
    print_every = 200          # 200サンプル(約0.1秒)ごとに画面表示。printは重いので毎サンプルは行わない
    t_report = time.monotonic()

    CALIBRATION_SEC = 3.0
    ST_CALIB = 0
    ST_LOOP = 1
    calib_flag = ST_CALIB
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    s_roll = 0.0
    s_pitch = 0.0
    s_yaw = 0.0
    draft_roll = 0.0
    draft_pitch = 0.0
    draft_yaw = 0.0
    calib_flag = ST_CALIB
    START_TIME = time.time()

    try:
        while True:
            elapsed = time.time() - START_TIME
            n = imu.fifo_unread_count()
            if n < 2:
                time.sleep(0.0005)
                continue

            n_read = min(n, MAX_FRAMES_PER_TRANSFER)
            raw = imu.burst_read_fifo(n_read)

            for i in range(n_read):
                off = i * FRAME_SIZE
                tag, x, y, z = parse_fifo_frame(raw, off)

                updated = update_from_frame(cf, tag, x, y, z)

                # -----------------------------
                # キャリブレーション処理（修正版）
                # -----------------------------
                
                if calib_flag == ST_CALIB and updated:
                    roll, pitch, yaw = cf.angles()

                    # 積分角度を合計する（正しい式）
                    s_roll  += roll - s_roll
                    s_pitch += pitch - s_pitch
                    s_yaw   += yaw - s_yaw

                    if elapsed >= CALIBRATION_SEC:
                        drift_roll  = s_roll  / elapsed
                        drift_pitch = s_pitch / elapsed
                        drift_yaw   = s_yaw   / elapsed

                        print(f"Calib Result: sampling_time={elapsed:.3f}s")
                        print(f"  drift_roll:  {drift_roll:.6f} °/s")
                        print(f"  drift_pitch: {drift_pitch:.6f} °/s")
                        print(f"  drift_yaw:   {drift_yaw:.6f} °/s")
                        print("---")

                        calib_flag = ST_LOOP

                # -----------------------------
                # 通常処理（あなたの元コード）
                # -----------------------------
                if updated:
                    sample_count += 1

                    if sample_count % print_every == 0:
                        roll, pitch, yaw = cf.angles()

                        roll - draft_roll * elapsed
                        pitch - draft_pitch * elapsed
                        yaw - draft_yaw * elapsed

                        now = time.monotonic()
                        rate = print_every / (now - t_report) if now > t_report else 0.0
                        t_report = now
                        print(f"roll={roll:7.2f} pitch={pitch:7.2f} yaw={yaw:7.2f}  "
                            f"[{rate:.0f} sample/s]")

    except KeyboardInterrupt:
        pass
    finally:
        imu.close()


def parse_fifo_frame(raw, off):
    """
    FIFO 1フレーム(7byte)を解析して tag, x, y, z を返す。
    raw: burst_read_fifo() の bytes
    off: フレーム先頭インデックス
    """
    tag = raw[off] >> 3  # TAG_SENSOR_[4:0]

    x = to_int16(raw[off + 1], raw[off + 2])
    y = to_int16(raw[off + 3], raw[off + 4])
    z = to_int16(raw[off + 5], raw[off + 6])

    return tag, x, y, z


def update_from_frame(cf, tag, x, y, z):
    """
    TAGに応じて相補フィルタを更新する。
    """
    if tag == TAG_ACCEL_NC:
        ax_g = x * ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0
        ay_g = y * ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0
        az_g = z * ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0
        cf.update_accel(ax_g, ay_g, az_g)

    elif tag == TAG_GYRO_NC:
        gx_dps = x * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        gy_dps = y * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        gz_dps = z * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        cf.update_gyro(gx_dps, gy_dps, gz_dps)
        return True  # ジャイロ更新時のみサンプル数を増やす

    return False


if __name__ == "__main__":
    main()