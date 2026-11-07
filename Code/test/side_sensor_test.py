#!/usr/bin/env python3
"""TCA9548A CH2=左、CH3=右のVL53L1Xを確認する単独テスト。"""

import time
from collections import deque
from statistics import median

import adafruit_vl53l1x
import board
import busio

TCA_ADDR = 0x70
LEFT_CHANNEL = 2
RIGHT_CHANNEL = 3
MAX_DISTANCE_CM = 10.0
DISTANCE_OFFSET_CM = 2.0
TIMING_BUDGET_MS = 20
FILTER_SAMPLES = 3
STALE_SECONDS = 0.5
TEST_SECONDS = 15.0


class SideSensors:
    def __init__(self) -> None:
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensors = {}
        self.values = {"LEFT": None, "RIGHT": None}
        self.samples = {
            "LEFT": deque(maxlen=FILTER_SAMPLES),
            "RIGHT": deque(maxlen=FILTER_SAMPLES),
        }
        self.updated = {"LEFT": 0.0, "RIGHT": 0.0}
        self.channels = {"LEFT": LEFT_CHANNEL, "RIGHT": RIGHT_CHANNEL}

    def write_mux(self, mask: int) -> None:
        deadline = time.monotonic() + 1.0
        while not self.i2c.try_lock():
            if time.monotonic() >= deadline:
                raise TimeoutError("I2C bus lock timeout")
            time.sleep(0.001)
        try:
            self.i2c.writeto(TCA_ADDR, bytes((mask,)))
        finally:
            self.i2c.unlock()

    def select(self, channel: int) -> None:
        # 前のチャネルを確実に切り、同一アドレスの2台が同時に見えないようにする。
        self.write_mux(0x00)
        time.sleep(0.002)
        self.write_mux(1 << channel)
        time.sleep(0.010)

    def create_sensor(self, side: str, channel: int):
        last_error = None
        for attempt in range(1, 4):
            try:
                self.select(channel)
                sensor = adafruit_vl53l1x.VL53L1X(self.i2c)
                sensor.distance_mode = 1
                sensor.timing_budget = TIMING_BUDGET_MS
                print(f"{side} CH{channel} initialized", flush=True)
                return sensor
            except RuntimeError as error:
                last_error = error
                print(
                    f"{side} CH{channel} init retry {attempt}/3: {error}",
                    flush=True,
                )
                time.sleep(0.05)
        raise RuntimeError(f"{side} CH{channel} initialization failed: {last_error}")

    def start(self) -> None:
        # 片側を測距中にしたまま、もう片側を初期化しない。
        # まず両方のID確認・設定を完了する。
        for side, channel in self.channels.items():
            self.sensors[side] = self.create_sensor(side, channel)
        # 両方の初期化後、チャネルごとに測距を開始する。
        for side, channel in self.channels.items():
            self.select(channel)
            self.sensors[side].start_ranging()
            time.sleep(0.03)
            print(f"{side} CH{channel} ranging", flush=True)

    def read(self) -> tuple[float | None, float | None]:
        for side, channel in self.channels.items():
            self.select(channel)
            sensor = self.sensors[side]
            if hasattr(sensor, "data_ready") and not sensor.data_ready:
                continue
            value = sensor.distance
            if hasattr(sensor, "clear_interrupt"):
                sensor.clear_interrupt()
            # 下限は設けず、上限だけ10 cmへ丸める。
            # 測距不能はセンサー異常の可能性があるためNoneのままにする。
            if value is None:
                self.samples[side].clear()
                self.values[side] = None
                self.updated[side] = 0.0
                continue
            measured = min(
                max(float(value) + DISTANCE_OFFSET_CM, 0.0),
                MAX_DISTANCE_CM,
            )
            self.samples[side].append(measured)
            self.values[side] = float(median(self.samples[side]))
            self.updated[side] = time.monotonic()

        now = time.monotonic()
        output = []
        for side in ("LEFT", "RIGHT"):
            value = self.values[side]
            if value is None or now - self.updated[side] > STALE_SECONDS:
                output.append(None)
            else:
                output.append(value)
        return output[0], output[1]

    def close(self) -> None:
        for side, channel in self.channels.items():
            sensor = self.sensors.get(side)
            if sensor is None:
                continue
            try:
                self.select(channel)
                sensor.stop_ranging()
            except Exception:
                pass
        try:
            self.write_mux(0x00)
        finally:
            if hasattr(self.i2c, "deinit"):
                self.i2c.deinit()


def main() -> int:
    print("side_sensor_test.py started", flush=True)
    print("CH2=LEFT, CH3=RIGHT", flush=True)
    sensors = SideSensors()
    try:
        sensors.start()
        deadline = time.monotonic() + TEST_SECONDS
        last_displayed = None
        while time.monotonic() < deadline:
            left, right = sensors.read()
            displayed = (
                None if left is None else round(left, 1),
                None if right is None else round(right, 1),
            )
            if displayed != last_displayed:
                left_text = "---" if left is None else f"{displayed[0]:5.1f}"
                right_text = "---" if right is None else f"{displayed[1]:5.1f}"
                print(f"LEFT={left_text} cm | RIGHT={right_text} cm", flush=True)
                last_displayed = displayed
            time.sleep(0.01)
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}", flush=True)
        return 1
    finally:
        sensors.close()
        print("Finished", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
