#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from smbus2 import SMBus
import board
import busio
import adafruit_vl53l1x

# ---------- I2C アドレス ----------
TCA_ADDR = 0x70
TCS_ADDR = 0x29     # TCS34725
VL_ADDR  = 0x29     # VL53L1X（チャンネルが違うのでOK）

# ---------- TCA9548A ----------
def tca_select(bus, ch):
    bus.write_byte(TCA_ADDR, 1 << ch)

# ---------- TCS34725 ----------
def tcs_init(bus):
    bus.write_byte_data(TCS_ADDR, 0x80, 0x03)
    time.sleep(0.01)

def tcs_read(bus):
    c = bus.read_word_data(TCS_ADDR, 0x94)
    r = bus.read_word_data(TCS_ADDR, 0x96)
    g = bus.read_word_data(TCS_ADDR, 0x98)
    b = bus.read_word_data(TCS_ADDR, 0x9A)
    return c, r, g, b

# ================================================================
# メイン処理
# ================================================================
def main():
    bus = SMBus(1)
    i2c = busio.I2C(board.SCL, board.SDA)

    print("=== TCA9548A センサーテスト開始 ===")

    # ---------- カラーセンサー初期化 ----------
    for ch in (0, 1):
        try:
            tca_select(bus, ch)
            tcs_init(bus)
            print(f"[TCS34725 CH{ch}] 初期化 OK")
        except Exception as e:
            print(f"[TCS34725 CH{ch}] 初期化失敗:", e)

    # ---------- 距離センサー初期化 ----------
    vl = {}
    for ch in (2, 3):
        try:
            tca_select(bus, ch)
            vl[ch] = adafruit_vl53l1x.VL53L1X(i2c)
            vl[ch].start_ranging()
            print(f"[VL53L1X CH{ch}] 初期化 OK")
        except Exception as e:
            print(f"[VL53L1X CH{ch}] 初期化失敗:", e)

    print("\n=== センサー読み取り開始 ===\n")

    while True:
        # ---------- カラーセンサー ----------
        for ch in (0, 1):
            try:
                tca_select(bus, ch)
                c, r, g, b = tcs_read(bus)
                print(f"COLOR CH{ch}: C={c} R={r} G={g} B={b}")
            except Exception as e:
                print(f"COLOR CH{ch}: 読み取り失敗 ({e})")

        # ---------- 距離センサー ----------
        for ch in (2, 3):
            try:
                tca_select(bus, ch)
                mm = vl[ch].distance
                if mm is not None:
                    mm = mm * 10.0  # cm → mm
                print(f"DIST CH{ch}: {mm} mm")
            except Exception as e:
                print(f"DIST CH{ch}: 読み取り失敗 ({e})")

        print("-" * 40)
        time.sleep(0.3)


if __name__ == "__main__":
    main()
