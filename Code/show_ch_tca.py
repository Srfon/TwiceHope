#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smbus2
import time

BUS_NUM = 1
MUX_ADDR = 0x70   # TCA9548A の I2C アドレス

bus = smbus2.SMBus(BUS_NUM)

def select_channel(ch):
    """
    TCA9548A のチャネルを選択する。
    ch = 0〜7
    """
    if 0 <= ch <= 7:
        bus.write_byte(MUX_ADDR, 1 << ch)
        time.sleep(0.002)  # 切り替え安定化のため少し待つ
    else:
        raise ValueError("Channel must be 0-7")

def scan_i2c_devices():
    """
    現在選択されているチャネルで見える I2C デバイスをスキャンする。
    """
    devices = []
    for addr in range(0x03, 0x78):  # 有効アドレス範囲
        try:
            bus.write_byte(addr, 0x00)
            devices.append(addr)
        except:
            pass
    return devices

def main():
    print("TCA9548A I2C Scan (CH0〜CH7)")
    print("---------------------------")

    for ch in range(8):
        select_channel(ch)
        devices = scan_i2c_devices()

        print(f"CH{ch}: ", end="")
        if devices:
            print(", ".join(f"0x{d:02X}" for d in devices))
        else:
            print("No device")

    # 最後に全チャネル OFF に戻す
    bus.write_byte(MUX_ADDR, 0x00)
    bus.close()

if __name__ == "__main__":
    main()
