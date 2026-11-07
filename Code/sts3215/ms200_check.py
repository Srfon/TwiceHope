#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS200のUARTだけを確認する。モーターとWebサーバーは使用しない。"""

from __future__ import annotations

import argparse
import time

import serial


MS200_START_COMMAND = bytes.fromhex("A5 F5 A2 C1 01 81 B3 31 F2")
MS200_FRAME_SIZE = 47
MS200_HEADER = bytes((0x54, 0x2C))


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (
                ((value << 1) ^ 0x4D) & 0xFF
                if value & 0x80
                else (value << 1) & 0xFF
            )
    return value


def run(port: str, baudrate: int, seconds: float) -> None:
    print(f"open: port={port}, baud={baudrate}")
    total_bytes = 0
    header_count = 0
    valid_frames = 0
    crc_errors = 0
    first_bytes = bytearray()

    with serial.Serial(port, baudrate, timeout=0.05) as device:
        device.reset_input_buffer()
        device.write(MS200_START_COMMAND)
        device.flush()
        print(f"start command sent: {MS200_START_COMMAND.hex(' ')}")

        deadline = time.monotonic() + seconds
        next_report = time.monotonic() + 1.0
        buffer = bytearray()
        while time.monotonic() < deadline:
            chunk = device.read(max(1, device.in_waiting))
            if chunk:
                total_bytes += len(chunk)
                buffer.extend(chunk)
                if len(first_bytes) < 64:
                    first_bytes.extend(chunk[:64 - len(first_bytes)])

            while True:
                header_index = buffer.find(MS200_HEADER)
                if header_index < 0:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break
                if header_index:
                    del buffer[:header_index]
                if len(buffer) < MS200_FRAME_SIZE:
                    break
                header_count += 1
                frame = bytes(buffer[:MS200_FRAME_SIZE])
                if crc8(frame[:-1]) == frame[-1]:
                    valid_frames += 1
                    del buffer[:MS200_FRAME_SIZE]
                else:
                    crc_errors += 1
                    del buffer[0]

            now = time.monotonic()
            if now >= next_report:
                print(
                    f"bytes={total_bytes}, headers={header_count}, "
                    f"valid={valid_frames}, crc_errors={crc_errors}, "
                    f"waiting={device.in_waiting}"
                )
                next_report = now + 1.0

    print("--- result ---")
    print(
        f"bytes={total_bytes}, headers={header_count}, "
        f"valid={valid_frames}, crc_errors={crc_errors}"
    )
    print(f"first_bytes={first_bytes.hex(' ') if first_bytes else '(none)'}")
    if total_bytes == 0:
        print("判定: UARTから1バイトも来ていません。ポート、競合、電源、TX/RX/GNDを確認してください。")
    elif valid_frames == 0:
        print("判定: バイトは来ていますがMS200正常フレームではありません。baudrateか接続先を確認してください。")
    else:
        print("判定: MS200の正常フレームを受信できています。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MS200 UART受信診断")
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seconds", type=float, default=8.0)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(args.port, args.baud, args.seconds)
