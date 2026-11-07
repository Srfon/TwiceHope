#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LSM6DSV16X (秋月電子 6軸IMUセンサーモジュール, akizukidenshi.com/catalog/g/g130950/)
Raspberry Pi 5 / I2C1 (/dev/i2c-1) 用ドライバ

FIFOをContinuousモードで動作させ、UIチャンネル(ジャイロ+加速度)に加えて
EIS(Enhanced Electronic Image Stabilization)チャンネルのジャイロ出力も同一FIFOに
バッチングし、1回のI2Cトランザクションでバースト読み出しした上で、
UI/EIS両チャンネルのジャイロ値を融合してから相補フィルタでロール/ピッチ/ヨー角を更新する。

--- UI / EIS / OIS の3チャンネルについて (今回はUI+EISのみ実装) ---
LSM6DSV16Xは同一の物理MEMSジャイロ素子に対して独立した3つの信号処理コアを持つ
("triple-channel architecture"):
  - UIチャンネル : 主インターフェース(I2C/SPI/I3C)経由。本ドライバの基本経路。
  - EISチャンネル: 主インターフェース経由でFIFOにも出力可能。ODRは1.92kHz/960Hzの2択のみ、
                   フルスケール・LPF1がUIとは独立に設定できる。
  - OISチャンネル: 補助SPI(SCX/SDXピンをmode3/4に切替えて使用)経由でのみ読み出し可能。
                   I2Cの主インターフェースからは一切読めない。
                   -> 本モジュール(g130950)はI2C1のみ配線されている前提のため、
                      OISは未実装。将来的にSCX/SDXを補助SPIとして追加配線すれば拡張可能。

--- EIS融合による精度改善の限界について(誠実な注記) ---
UI・EISは"別々の物理センサ"ではなく、同一MEMS共振子を読む独立回路(独立ADC/独立デジタル
フィルタ)に過ぎない。そのため:
  - ADC量子化ノイズ・回路由来の電子雑音はUI/EIS間でほぼ独立とみなせる
    -> 2サンプルの単純平均で、この成分の標準偏差は理論上最大 1/sqrt(2) 程度まで低減しうる。
  - MEMS共振子由来の機械的ノイズ(角度ランダムウォーク/ARW、バイアス不安定性)はUI/EISで
    共有されるため、平均化しても長期的なヨードリフトはほぼ改善しない。
  -> つまりEIS融合は「短期的な高周波ノイズの低減」には効くが、「長期ドリフトの根本解決」には
     ならない。長期ドリフトの補正は従来通り加速度センサによるロール/ピッチ補正、および
     必要であれば地磁気センサ等の絶対方位基準の追加が必要。

参照: LSM6DSV16Xデータシート (DS13510 Rev.4) および
      AN5763 (LSM6DSV16X: 6-axis IMU with embedded sensor fusion, AI, Qvar)
  - I2Cスレーブアドレス: 5.1.2節 Table12 (SAD=110101xb, SDO/SA0=GND時 0x6A)
  - レジスタマップ: 9章 Register mapping table
  - FIFO動作: 6.12節, FIFO_CTRL1-4 (07h-0Ah): 9.5-9.8節
  - FIFO_STATUS1/2 (1Bh/1Ch): 9.25-9.26節
  - FIFO_DATA_OUT_TAG (78h) 以降とTAGコード表: 9.84節 Table218
  - CTRL1/CTRL2 (ODR): 9.14-9.15節, CTRL6 (ジャイロFS): 9.19節, CTRL8 (加速度FS): 9.21節
  - CTRL7 (16h) LPF1_G_EN: 9.20節
  - CTRL_EIS (6Bh) ODR_G_EIS/FS_G_EIS/LPF_G_EIS_BW/G_EIS_ON_G_OIS_OUT_REG: AN5763 3.6節
  - FIFO_CTRL4 (0Ah) G_EIS_FIFO_EN(bit3): AN5763 3.6節、
    および STMicroelectronics/lsm6dsv16x-pid (lsm6dsv16x_reg.h) の
    lsm6dsv16x_fifo_ctrl4_t / lsm6dsv16x_ctrl_eis_t ビットフィールド定義で
    ビット位置を実装レベルで確認済み。
  - FIFO TAGコード 0x1E = GY_ENHANCED_EIS: STMicroelectronics/lsm6dsv-pid
    (lsm6dsv_reg.c, lsm6dsv_fifo_out_raw_get() のswitch文) で確認済み。
    LSM6DSVとLSM6DSV16Xは当該箇所のレジスタマップ・TAGテーブルが同一。

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
REG_CTRL6              = 0x15   # ジャイロ フルスケール, LPF1帯域 (UIチャンネル)
REG_CTRL7              = 0x16   # LPF1_G_EN 等
REG_CTRL8              = 0x17   # 加速度 フルスケール
REG_FIFO_STATUS1       = 0x1B
REG_FIFO_STATUS2       = 0x1C
REG_FIFO_DATA_OUT_TAG  = 0x78   # 以降 0x79-0x7E に X_L,X_H,Y_L,Y_H,Z_L,Z_H が続く(1フレーム7byte)
REG_CTRL_EIS           = 0x6B   # EISチャンネル ODR/FS/LPF設定 (AN5763 3.6節)

WHO_AM_I_VALUE = 0x70

# FIFO_DATA_OUT_TAG の TAG_SENSOR_[4:0] コード (Table 218)
TAG_GYRO_NC  = 0x01   # UIチャンネル ジャイロ (no compression)
TAG_ACCEL_NC = 0x02   # UIチャンネル 加速度 (no compression)
TAG_GYRO_EIS = 0x1E   # EISチャンネル ジャイロ (GY_ENHANCED_EIS)

# ------------------------------------------------------------------
# センサ設定値 (ODR=1.92kHz, ジャイロ±2000dps, 加速度±8g)
# ODR_XL/ODR_G = 1010b = 1.92kHz (Table 52 / Table 55)
# ------------------------------------------------------------------
ODR_CODE_1920HZ = 0b1010
GYRO_FS_2000DPS = 0b0100          # CTRL6 FS_G[3:0] (9.19節)
ACCEL_FS_8G     = 0b10            # CTRL8 FS_XL[1:0] (9.21節)

# CTRL_EIS(6Bh) 用設定値。ビット配置(LSB->MSB):
#   [2:0] FS_G_EIS   : EISチャンネルのジャイロフルスケール(UIのFS_G[3:0]と同じコード体系、3bit幅)
#   [3]   G_EIS_ON_G_OIS_OUT_REG
#         0='EISデータはFIFO/UI出力レジスタ経由で読む'(本実装で使う設定)
#         1='EISデータはOIS(補助SPI)出力レジスタ経由でのみ読む' -> I2Cからは読めなくなるので0固定
#   [4]   LPF_G_EIS_BW : EISチャンネル用LPF1の帯域選択(1bit=2択)。
#         具体的なカットオフ周波数(Hz)はデータシート未確認のため、
#         UIチャンネルと同じ「最広帯域/低遅延優先」の意図で0を選択する。
#   [5]   未使用
#   [7:6] ODR_G_EIS   : EISチャンネル有効化/ODR選択。00=無効(デフォルト), 01=1.92kHz, 10=960Hz
#         (この2値以外は選べない = UIチャンネルのような多段ODR選択はできない)
FS_G_EIS_2000DPS        = 0b100   # UIのGYRO_FS_2000DPSと同じ数値(3bit幅でも収まる)
G_EIS_ON_G_OIS_OUT_REG  = 0b0     # 0固定: I2C(UI)側からEISデータを読むため
LPF_G_EIS_BW_SEL        = 0b0     # 帯域選択(詳細特性未確認、UI同様に低遅延側を仮定)
ODR_G_EIS_OFF           = 0b00
ODR_G_EIS_1920HZ        = 0b01
ODR_G_EIS_960HZ         = 0b10

FRAME_SIZE = 7                     # TAG(1) + X_L/X_H/Y_L/Y_H/Z_L/Z_H(6)
# UI加速度+UIジャイロ+EISジャイロの3ストリームがFIFOに積まれるため、
# 1バーストあたりの読み出しフレーム数上限を従来の8から12へ引き上げ、
# 同じI2Cポーリング間隔でもFIFOオーバーランしにくいマージンを確保する。
MAX_FRAMES_PER_TRANSFER = 12       # 1回のI2Cトランザクションで読むフレーム数上限(安全マージン)


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

        # CTRL2(11h): ジャイロ(UIチャンネル) OP_MODE=high-performance(000), ODR=1.92kHz
        self.write_reg(REG_CTRL2, (0b000 << 4) | ODR_CODE_1920HZ)

        # CTRL6(15h): ジャイロ(UI)LPF1帯域=000(最広帯域, 低遅延優先), FS_G=±2000dps
        self.write_reg(REG_CTRL6, (0b000 << 4) | GYRO_FS_2000DPS)

        # CTRL7(16h): LPF1_G_EN=0。
        # データシート注記:「gyroscope OIS or EIS chainが有効な間はUIチェーンのLPF1は
        # 使用しない(使用不可)ことが推奨される」ため、EISを有効化するにあたり
        # リセット時のデフォルト値(=無効)に依存せず明示的に0を書き込んでおく。
        self.write_reg(REG_CTRL7, 0x00)

        # CTRL8(17h): 加速度LPF2/HPF=デフォルト(バイパス, ODR/2), FS_XL=±8g
        self.write_reg(REG_CTRL8, ACCEL_FS_8G)

        # CTRL_EIS(6Bh): EISチャンネルを1.92kHzで有効化。
        # FS_G_EISはUIチャンネルと同じ±2000dpsに揃え、両チャンネルの
        # LSB->dps変換係数(GYRO_SENSITIVITY_MDPS_PER_LSB)を共通化する。
        self.write_reg(
            REG_CTRL_EIS,
            (ODR_G_EIS_1920HZ << 6)
            | (LPF_G_EIS_BW_SEL << 4)
            | (G_EIS_ON_G_OIS_OUT_REG << 3)
            | FS_G_EIS_2000DPS,
        )

        # FIFO_CTRL3(09h): BDR_GY=1.92kHz(bit7:4), BDR_XL=1.92kHz(bit3:0)
        # (UIチャンネルのバッチレート。EISチャンネルのFIFOバッチ有効化はFIFO_CTRL4で行う)
        self.write_reg(REG_FIFO_CTRL3, (ODR_CODE_1920HZ << 4) | ODR_CODE_1920HZ)

        # FIFO_CTRL2(08h): 圧縮なし, STOP_ON_WTM無効(継続動作)
        self.write_reg(REG_FIFO_CTRL2, 0x00)

        # FIFOをいったんBypassにしてクリアしてからContinuousへ (データシート 6.12.2節の手順)
        self.write_reg(REG_FIFO_CTRL4, 0b000)          # FIFO_MODE=000 bypass (G_EIS_FIFO_EN含め全クリア)
        time.sleep(0.001)
        # FIFO_CTRL4(0Ah) ビット配置(LSB->MSB):
        #   [2:0] FIFO_MODE      : 110 = Continuous mode
        #   [3]   G_EIS_FIFO_EN  : 1 = EISチャンネルのジャイロ出力もFIFOにバッチングする
        #                          (専用TAG=0x1Eとタイムスタンプ付きで積まれる)
        #   [5:4] ODR_T_BATCH    : 00 = 温度センサのFIFOバッチングは無効(未使用のため)
        #   [7:6] DEC_TS_BATCH   : 00 = タイムスタンプのFIFOバッチングは無効
        #                          (本実装ではUI/EISのペアリングをFIFO到着順の
        #                          単純対応付けで行うため、ハードウェアタイムスタンプは
        #                          使用しない。厳密な同時刻対応が必要な場合は
        #                          DEC_TS_BATCHを01等にしてTIMESTAMP(TAG=0x04)フレームも
        #                          読み、タイムスタンプでUI/EISを突き合わせる拡張が可能)
        fifo_ctrl4 = (0b00 << 6) | (0b00 << 4) | (1 << 3) | 0b110
        self.write_reg(REG_FIFO_CTRL4, fifo_ctrl4)

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
        UI加速度・UIジャイロ・EISジャイロのいずれのフレームも同じ7byte構造
        (TAG(1)+XYZ(6))で積まれるため、読み出し手順自体はTAG種別によらず共通。
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


class GyroEisFusion:
    """
    UIチャンネルとEISチャンネルのジャイロ値を融合するためのヘルパー。

    両チャンネルは同一の物理MEMSジャイロ素子を読む独立回路(独立ADC/独立デジタル
    フィルタチェーン)であり、別々の物理センサではない。したがって:
      - 電子回路由来の量子化/熱雑音 -> UI/EIS間でほぼ独立とみなせるので、
        単純平均で標準偏差が理論上最大 1/sqrt(2) 程度まで低減しうる。
      - MEMS機械振動子由来のノイズ(ARW, バイアス不安定性) -> UI/EISで共有されるため、
        平均化しても長期ドリフトはほとんど改善しない。

    ハードウェアタイムスタンプ(TIMESTAMPフレーム, TAG=0x04)による厳密な同時刻対応付けは
    行わず、「直近に届いた未消費のEISサンプルを、次に届いたUIジャイロサンプルと
    1回だけ平均して消費する」という簡易的な到着順ペアリングを行う。
    UI/EISは同一ODR(1.92kHz)で駆動しているため、通常は同じバースト読み出し内で
    ほぼ同時に両方のフレームが届く。
    """

    def __init__(self):
        self._eis = None  # (gx_dps, gy_dps, gz_dps) or None

    def push_eis(self, gx_dps, gy_dps, gz_dps):
        self._eis = (gx_dps, gy_dps, gz_dps)

    def fuse_with_ui(self, gx_dps, gy_dps, gz_dps):
        if self._eis is None:
            # 起動直後などEISサンプルがまだ届いていない場合はUI値のみを使う
            return gx_dps, gy_dps, gz_dps
        ex, ey, ez = self._eis
        self._eis = None  # 使い捨て: 同じEISサンプルを次のUIサンプルと再び平均しない
        return (
            (gx_dps + ex) / 2.0,
            (gy_dps + ey) / 2.0,
            (gz_dps + ez) / 2.0,
        )


GYRO_SENSITIVITY_MDPS_PER_LSB  = 70.0     # ±2000dps時 (Mechanical characteristics表)。UI/EIS共通
ACCEL_SENSITIVITY_MG_PER_LSB   = 0.244    # ±8g時


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
    eis_fusion = GyroEisFusion()

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
            if n < 3:
                # UI加速度+UIジャイロ+EISジャイロの3フレームが揃うまで待つ目安として閾値を2->3に変更
                time.sleep(0.0005)
                continue

            n_read = min(n, MAX_FRAMES_PER_TRANSFER)
            raw = imu.burst_read_fifo(n_read)

            for i in range(n_read):
                off = i * FRAME_SIZE
                tag, x, y, z = parse_fifo_frame(raw, off)

                updated = update_from_frame(cf, eis_fusion, tag, x, y, z)

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
    UI加速度(TAG=0x02)・UIジャイロ(TAG=0x01)・EISジャイロ(TAG=0x1E)いずれも
    同一の7byte構造なので、この関数はTAG種別によらず共通で使える。
    """
    tag = raw[off] >> 3  # TAG_SENSOR_[4:0]

    x = to_int16(raw[off + 1], raw[off + 2])
    y = to_int16(raw[off + 3], raw[off + 4])
    z = to_int16(raw[off + 5], raw[off + 6])

    return tag, x, y, z


def update_from_frame(cf, eis_fusion, tag, x, y, z):
    """
    TAGに応じて相補フィルタを更新する。

    TAG_ACCEL_NC : UI加速度 -> cf.update_accel() に反映するのみ(戻り値False)。
    TAG_GYRO_EIS : EISジャイロ -> eis_fusion に「未消費のEISサンプル」として保持するのみ
                   (この時点ではまだ相補フィルタは更新しない。戻り値False)。
    TAG_GYRO_NC  : UIジャイロ -> eis_fusion.fuse_with_ui() でEISサンプルと平均した上で
                   cf.update_gyro() に反映する。相補フィルタの1積分ステップは
                   このUIジャイロフレーム到着のタイミングでのみ進める
                   (EISフレーム到着のたびに二重積分しないための設計)。
                   戻り値True(sample_countのカウント対象)。
    """
    if tag == TAG_ACCEL_NC:
        ax_g = x * ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0
        ay_g = y * ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0
        az_g = z * ACCEL_SENSITIVITY_MG_PER_LSB / 1000.0
        cf.update_accel(ax_g, ay_g, az_g)

    elif tag == TAG_GYRO_EIS:
        gx_dps = x * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        gy_dps = y * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        gz_dps = z * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        eis_fusion.push_eis(gx_dps, gy_dps, gz_dps)

    elif tag == TAG_GYRO_NC:
        gx_dps = x * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        gy_dps = y * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        gz_dps = z * GYRO_SENSITIVITY_MDPS_PER_LSB / 1000.0
        fx_dps, fy_dps, fz_dps = eis_fusion.fuse_with_ui(gx_dps, gy_dps, gz_dps)
        cf.update_gyro(fx_dps, fy_dps, fz_dps)
        return True  # UIジャイロ更新(=EIS融合込みの1積分ステップ)時のみサンプル数を増やす

    return False


if __name__ == "__main__":
    main()