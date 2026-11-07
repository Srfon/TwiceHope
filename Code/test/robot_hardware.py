"""STS3215 4WDと前方VL53L1Xを走行判断へ接続する実機層。"""

from __future__ import annotations

import argparse
from collections import deque
import threading
import time
import math
from statistics import median

import adafruit_vl53l1x
import board
import busio
import smbus2
from sts3215 import STS3215

try:
    # robot_hardware.pyとnavigation.pyを同じフォルダに置いて直接起動する場合
    from navigation import MotionCommand
except ModuleNotFoundError as error:
    if error.name != "navigation":
        raise
    # `python -m ms200_viewer...` で起動する場合
    from ms200_viewer.navigation import MotionCommand

MOTOR_PORT = "/dev/ttyUSB0"
MOTOR_BAUDRATE = 1_000_000
LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR = 1, 2, 3, 4
MOTOR_IDS = (LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR)
MOTOR_DIRECTION = {LEFT_FRONT: 1, RIGHT_FRONT: -1, LEFT_REAR: 1, RIGHT_REAR: -1}

# 自律走行はキーボード操作時より低い値から調整する。
FORWARD_POWER = 1000
TURN_POWER = 1000
POWER_MAX = 4095

TCA_ADDR = 0x70
VL53_CHANNEL_MASK = 0x04
LEFT_VL53_CHANNEL = 2
RIGHT_VL53_CHANNEL = 3
EMERGENCY_DISTANCE_CM = 5.0
SENSOR_STALE_SECONDS = 0.5
MIN_VALID_DISTANCE_CM = 3.0
SIDE_MAX_DISTANCE_CM = 10.0
SIDE_DISTANCE_OFFSET_CM = 2.0
SIDE_TIMING_BUDGET_MS = 20
SIDE_FILTER_SAMPLES = 3

# 車輪・エンコーダー設定。モーター軸と車輪が直結なら比率は1.0。
WHEEL_DIAMETER_MM = 80.0
ENCODER_COUNTS_PER_REV = 4096
MOTOR_REVS_PER_WHEEL_REV = 1.0
WHEEL_CIRCUMFERENCE_MM = math.pi * WHEEL_DIAMETER_MM
MM_PER_MOTOR_COUNT = (
    WHEEL_CIRCUMFERENCE_MM
    / (ENCODER_COUNTS_PER_REV * MOTOR_REVS_PER_WHEEL_REV)
)


def signed_to_raw(value: int) -> int:
    magnitude = max(0, min(POWER_MAX, abs(int(value))))
    return magnitude | 0x8000 if value < 0 else magnitude


class FourWheelDrive:
    """添付の動作確認済みコードと同じSTS3215制御。"""

    def __init__(self, servo: STS3215, power: int = FORWARD_POWER) -> None:
        self.servo = servo
        self.power = power
        self.command = MotionCommand.STOP
        self.motion = "STOP"

    def initialize(self) -> None:
        try:
            for motor_id in MOTOR_IDS:
                if not self.servo.ping(motor_id):
                    raise RuntimeError(f"Servo ID={motor_id} not found")
            for motor_id in MOTOR_IDS:
                self.servo.torque(motor_id, False)
                self.servo.set_mode(motor_id, STS3215.MODE_VELOCITY)
                self.servo.torque(motor_id, True)
            self.stop(force=True)
        except Exception:
            self.shutdown()
            raise

    def _set_motor_speed(self, motor_id: int, speed: int) -> None:
        corrected = speed * MOTOR_DIRECTION[motor_id]
        self.servo.write_u16(motor_id, self.servo.ADDR_GOAL_SPEED, signed_to_raw(corrected))

    def _drive(self, left_speed: int, right_speed: int) -> None:
        self._set_motor_speed(LEFT_FRONT, left_speed)
        self._set_motor_speed(LEFT_REAR, left_speed)
        self._set_motor_speed(RIGHT_FRONT, right_speed)
        self._set_motor_speed(RIGHT_REAR, right_speed)

    def forward(self) -> None:
        self._drive(self.power, self.power)
        self.command = MotionCommand.FORWARD
        self.motion = "FORWARD"

    def backward(self) -> None:
        self._drive(-self.power, -self.power)
        self.motion = "BACKWARD"

    def left(self) -> None:
        self._drive(TURN_POWER, -TURN_POWER)
        self.command = MotionCommand.TURN_LEFT
        self.motion = "LEFT"

    def right(self) -> None:
        self._drive(-TURN_POWER, TURN_POWER)
        self.command = MotionCommand.TURN_RIGHT
        self.motion = "RIGHT"

    def stop(self, force: bool = False) -> None:
        if force or self.command != MotionCommand.STOP:
            self._drive(0, 0)
        self.command = MotionCommand.STOP
        self.motion = "STOP"

    def apply(self, command: MotionCommand, force: bool = False) -> None:
        if not force and command == self.command:
            return
        if command == MotionCommand.FORWARD:
            self.forward()
        elif command == MotionCommand.TURN_LEFT:
            self.left()
        elif command == MotionCommand.TURN_RIGHT:
            self.right()
        else:
            self.stop(force=force)

    def shutdown(self) -> None:
        try:
            self.stop(force=True)
        finally:
            for motor_id in MOTOR_IDS:
                try:
                    self.servo.torque(motor_id, False)
                except Exception:
                    pass


def open_default_drive() -> tuple[STS3215, FourWheelDrive]:
    """添付コードと同じ順番で通信を開き、4輪を初期化する。"""
    servo = STS3215(port=MOTOR_PORT, baudrate=MOTOR_BAUDRATE, timeout=0.1)
    servo.open()
    drive = FourWheelDrive(servo)
    try:
        drive.initialize()
    except Exception:
        servo.close()
        raise
    return servo, drive


class WheelOdometry:
    """4個のSTS3215位置フィードバックから直進距離を積算する。"""

    def __init__(self, drive: FourWheelDrive) -> None:
        self.drive = drive
        self._previous: dict[int, int] = {}
        self.total_distance_mm = 0.0
        self._position_address = self._find_position_address()

    def _find_position_address(self) -> int:
        # 添付ライブラリの定数名がどちらでも動くようにする。
        for name in ("ADDR_PRESENT_POSITION", "ADDR_PRESENT_POS"):
            value = getattr(self.drive.servo, name, None)
            if value is not None:
                return int(value)
        raise RuntimeError(
            "sts3215.pyに現在位置アドレスがありません。"
            "ADDR_PRESENT_POSITIONまたはADDR_PRESENT_POSを確認してください"
        )

    def _read_position(self, motor_id: int) -> int:
        reader = getattr(self.drive.servo, "read_u16", None)
        if reader is None:
            raise RuntimeError("sts3215.pyにread_u16()が必要です")
        value = reader(motor_id, self._position_address)
        if value is None:
            raise RuntimeError(f"Servo ID={motor_id} position read failed")
        return int(value) & 0x0FFF

    def reset(self) -> None:
        self._previous = {motor_id: self._read_position(motor_id) for motor_id in MOTOR_IDS}
        self.total_distance_mm = 0.0

    def update(self) -> float:
        if not self._previous:
            self.reset()
            return self.total_distance_mm

        wheel_deltas: list[float] = []
        for motor_id in MOTOR_IDS:
            current = self._read_position(motor_id)
            previous = self._previous[motor_id]
            # 4095→0、0→4095の周回を-2048..2047へ展開する。
            counts = (current - previous + 2048) % ENCODER_COUNTS_PER_REV - 2048
            self._previous[motor_id] = current
            forward_counts = counts * MOTOR_DIRECTION[motor_id]
            wheel_deltas.append(forward_counts * MM_PER_MOTOR_COUNT)

        # 旋回時は左右が相殺され、直進時は4輪の平均距離になる。
        self.total_distance_mm += sum(wheel_deltas) / len(wheel_deltas)
        return self.total_distance_mm


class FrontDistanceSensor:
    """TCA9548Aの0x04チャンネルにあるVL53L1Xをバックグラウンド測距する。"""

    def __init__(self) -> None:
        self._mux_bus: smbus2.SMBus | None = None
        self._sensor: adafruit_vl53l1x.VL53L1X | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._distance_cm: float | None = None
        self._updated_at = 0.0
        self._error: str | None = None

    def start(self) -> None:
        self._mux_bus = smbus2.SMBus(1)
        self._mux_bus.write_byte(TCA_ADDR, VL53_CHANNEL_MASK)
        i2c = busio.I2C(board.SCL, board.SDA)
        self._sensor = adafruit_vl53l1x.VL53L1X(i2c)
        self._sensor.start_ranging()
        self._thread = threading.Thread(target=self._run, name="front-vl53l1x", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        assert self._sensor is not None
        while not self._stop.wait(0.05):
            try:
                # 他の処理がマルチプレクサを切り替えても、読み取り前に戻す。
                assert self._mux_bus is not None
                self._mux_bus.write_byte(TCA_ADDR, VL53_CHANNEL_MASK)
                if hasattr(self._sensor, "data_ready") and not self._sensor.data_ready:
                    continue
                value = self._sensor.distance
                if hasattr(self._sensor, "clear_interrupt"):
                    self._sensor.clear_interrupt()
                with self._lock:
                    if value is not None and float(value) >= MIN_VALID_DISTANCE_CM:
                        self._distance_cm = float(value)
                        self._updated_at = time.monotonic()
                        self._error = None
                    else:
                        self._distance_cm = None
                        self._updated_at = 0.0
                        self._error = f"invalid distance: {value!r} cm"
            except Exception as error:
                with self._lock:
                    self._error = f"{type(error).__name__}: {error}"

    def distance_cm(self) -> float | None:
        with self._lock:
            value = self._distance_cm
            updated_at = self._updated_at
        if value is None or time.monotonic() - updated_at > SENSOR_STALE_SECONDS:
            return None
        return value

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._sensor:
            try:
                self._sensor.stop_ranging()
            except Exception:
                pass
        if self._mux_bus:
            self._mux_bus.close()


class SideDistanceSensors:
    """TCA9548A CH2=左、CH3=右のVL53L1Xを交互に測距する。"""

    def __init__(self) -> None:
        self._mux_bus: smbus2.SMBus | None = None
        self._i2c = None
        self._sensors: dict[str, adafruit_vl53l1x.VL53L1X] = {}
        self._channels = {"left": LEFT_VL53_CHANNEL, "right": RIGHT_VL53_CHANNEL}
        self._values: dict[str, float | None] = {"left": None, "right": None}
        self._samples = {
            "left": deque(maxlen=SIDE_FILTER_SAMPLES),
            "right": deque(maxlen=SIDE_FILTER_SAMPLES),
        }
        self._updated: dict[str, float] = {"left": 0.0, "right": 0.0}
        self._errors: dict[str, str | None] = {"left": None, "right": None}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def _select(self, channel: int) -> None:
        assert self._i2c is not None
        deadline = time.monotonic() + 1.0
        while not self._i2c.try_lock():
            if time.monotonic() >= deadline:
                raise TimeoutError("I2C bus lock timeout")
            time.sleep(0.001)
        try:
            self._i2c.writeto(TCA_ADDR, b"\x00")
        finally:
            self._i2c.unlock()
        time.sleep(0.002)
        deadline = time.monotonic() + 1.0
        while not self._i2c.try_lock():
            if time.monotonic() >= deadline:
                raise TimeoutError("I2C bus lock timeout")
            time.sleep(0.001)
        try:
            self._i2c.writeto(TCA_ADDR, bytes((1 << channel,)))
        finally:
            self._i2c.unlock()
        time.sleep(0.010)

    def _create_sensor(self, side: str, channel: int):
        last_error = None
        for attempt in range(1, 4):
            try:
                self._select(channel)
                sensor = adafruit_vl53l1x.VL53L1X(self._i2c)
                sensor.distance_mode = 1
                sensor.timing_budget = SIDE_TIMING_BUDGET_MS
                return sensor
            except RuntimeError as error:
                last_error = error
                time.sleep(0.05)
        raise RuntimeError(f"{side} CH{channel} initialization failed: {last_error}")

    def start(self) -> None:
        self._i2c = busio.I2C(board.SCL, board.SDA)
        for side, channel in self._channels.items():
            self._sensors[side] = self._create_sensor(side, channel)
        for side, channel in self._channels.items():
            self._select(channel)
            self._sensors[side].start_ranging()
            time.sleep(0.03)
        self._thread = threading.Thread(target=self._run, name="side-vl53l1x", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            for side, channel in self._channels.items():
                if self._stop.is_set():
                    return
                try:
                    self._select(channel)
                    sensor = self._sensors[side]
                    if hasattr(sensor, "data_ready") and not sensor.data_ready:
                        continue
                    value = sensor.distance
                    if hasattr(sensor, "clear_interrupt"):
                        sensor.clear_interrupt()
                    with self._lock:
                        if value is None:
                            self._samples[side].clear()
                            self._values[side] = None
                            self._updated[side] = 0.0
                            self._errors[side] = "no valid range"
                            continue
                        measured = min(
                            max(float(value) + SIDE_DISTANCE_OFFSET_CM, 0.0),
                            SIDE_MAX_DISTANCE_CM,
                        )
                        self._samples[side].append(measured)
                        self._values[side] = float(median(self._samples[side]))
                        self._updated[side] = time.monotonic()
                        self._errors[side] = None
                except Exception as error:
                    with self._lock:
                        self._values[side] = None
                        self._updated[side] = 0.0
                        self._errors[side] = f"{type(error).__name__}: {error}"
            self._stop.wait(0.001)

    def distances_cm(self) -> tuple[float | None, float | None]:
        now = time.monotonic()
        with self._lock:
            values = []
            for side in ("left", "right"):
                value = self._values[side]
                if value is None or now - self._updated[side] > SENSOR_STALE_SECONDS:
                    values.append(None)
                else:
                    values.append(value)
        return values[0], values[1]

    def errors(self) -> tuple[str | None, str | None]:
        with self._lock:
            return self._errors["left"], self._errors["right"]

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        for side, channel in self._channels.items():
            sensor = self._sensors.get(side)
            if sensor is None:
                continue
            try:
                self._select(channel)
                sensor.stop_ranging()
            except Exception:
                pass
        if self._i2c:
            try:
                deadline = time.monotonic() + 1.0
                while not self._i2c.try_lock():
                    if time.monotonic() >= deadline:
                        raise TimeoutError("I2C bus lock timeout")
                    time.sleep(0.001)
                try:
                    self._i2c.writeto(TCA_ADDR, b"\x00")
                finally:
                    self._i2c.unlock()
            finally:
                if hasattr(self._i2c, "deinit"):
                    self._i2c.deinit()


class SafeMotionBridge:
    """走行指令へ独立した前方距離インターロックを掛ける。"""

    def __init__(self, drive: FourWheelDrive, front_sensor: FrontDistanceSensor) -> None:
        self.drive = drive
        self.front_sensor = front_sensor

    def apply(self, requested: MotionCommand) -> MotionCommand:
        distance = self.front_sensor.distance_cm()
        if requested == MotionCommand.FORWARD:
            # Noneは未準備・通信断なので安全側で停止する。
            if distance is None or distance <= EMERGENCY_DISTANCE_CM:
                self.drive.stop()
                return MotionCommand.STOP
        self.drive.apply(requested)
        return requested

    def emergency_stop(self) -> None:
        self.drive.stop(force=True)


def _test_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="STS3215 4WD hardware test")
    parser.add_argument(
        "--forward-test",
        action="store_true",
        help="確認後、指定時間だけ低速で前進する",
    )
    parser.add_argument("--power", type=int, default=300)
    parser.add_argument("--seconds", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = _test_arguments()
    if not args.forward_test:
        print("このファイルは通常、他のプログラムから読み込んで使います。")
        print("短時間の前進確認: python3 robot_hardware.py --forward-test")
        return 0

    power = max(100, min(1000, args.power))
    seconds = max(0.1, min(2.0, args.seconds))
    print("警告: ロボットが前進します。車輪を浮かせるか、周囲を空けてください。")
    input(f"power={power}, time={seconds:.1f}s で開始します。Enterを押してください: ")

    servo: STS3215 | None = None
    drive: FourWheelDrive | None = None
    try:
        servo, drive = open_default_drive()
        drive.power = power
        print("FORWARD")
        drive.apply(MotionCommand.FORWARD, force=True)
        time.sleep(seconds)
        drive.stop(force=True)
        print("STOP")
        return 0
    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}")
        return 1
    finally:
        if drive is not None:
            drive.shutdown()
        if servo is not None:
            servo.close()


if __name__ == "__main__":
    raise SystemExit(main())
