"""STS3215 4WDと前方VL53L1Xを走行判断へ接続する実機層。"""

from __future__ import annotations

import threading
import time
import math

import adafruit_vl53l1x
import board
import busio
import smbus2
from sts3215 import STS3215

from .navigation import MotionCommand

MOTOR_PORT = "/dev/ttyUSB0"
MOTOR_BAUDRATE = 1_000_000
LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR = 1, 2, 3, 4
MOTOR_IDS = (LEFT_FRONT, RIGHT_FRONT, LEFT_REAR, RIGHT_REAR)
MOTOR_DIRECTION = {LEFT_FRONT: 1, RIGHT_FRONT: -1, LEFT_REAR: 1, RIGHT_REAR: -1}

# 自律走行はキーボード操作時より低い値から調整する。
FORWARD_POWER = 500
TURN_POWER = 350
POWER_MAX = 4095

TCA_ADDR = 0x70
VL53_CHANNEL_MASK = 0x04
EMERGENCY_DISTANCE_CM = 5.0
SENSOR_STALE_SECONDS = 0.5

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

    def apply(self, command: MotionCommand, force: bool = False) -> None:
        if not force and command == self.command:
            return
        if command == MotionCommand.FORWARD:
            self._drive(self.power, self.power)
        elif command == MotionCommand.TURN_LEFT:
            self._drive(TURN_POWER, -TURN_POWER)
        elif command == MotionCommand.TURN_RIGHT:
            self._drive(-TURN_POWER, TURN_POWER)
        else:
            self._drive(0, 0)
        self.command = command

    def stop(self, force: bool = False) -> None:
        self.apply(MotionCommand.STOP, force=force)

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
                value = self._sensor.distance
                with self._lock:
                    if value is not None:
                        self._distance_cm = float(value)
                        self._updated_at = time.monotonic()
                        self._error = None
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
