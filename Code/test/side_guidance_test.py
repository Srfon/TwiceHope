#!/usr/bin/env python3
"""左右VL53L1Xを使った短時間の側壁追従テスト。"""

import time

from side_sensor_test import SideSensors
from sts3215 import STS3215

PORT = "/dev/ttyUSB0"
BAUDRATE = 1_000_000
MOTOR_IDS = (1, 2, 3, 4)
MOTOR_DIRECTION = {1: 1, 2: -1, 3: 1, 4: -1}

POWER = 1000
MIN_CLEARANCE_CM = 5.0
CORRECTION_RANGE_CM = 10.0
CENTER_DEADBAND_CM = 1.0
MAX_CORRECTION = 0.45
CORRECTION_PER_CM = 0.08
MAX_RUN_SECONDS = 1.0


def signed_to_raw(value: int) -> int:
    magnitude = max(0, min(4095, abs(int(value))))
    return magnitude | 0x8000 if value < 0 else magnitude


class Drive:
    def __init__(self) -> None:
        self.servo = STS3215(port=PORT, baudrate=BAUDRATE, timeout=0.1)
        self.last_speeds = None
        self.opened = False

    def start(self) -> None:
        self.servo.open()
        self.opened = True
        for motor_id in MOTOR_IDS:
            if not self.servo.ping(motor_id):
                raise RuntimeError(f"Servo ID={motor_id} not found")
        for motor_id in MOTOR_IDS:
            self.servo.torque(motor_id, False)
            self.servo.set_mode(motor_id, STS3215.MODE_VELOCITY)
            self.servo.torque(motor_id, True)
        self.drive(0, 0, force=True)

    def set_speed(self, motor_id: int, speed: int) -> None:
        corrected = speed * MOTOR_DIRECTION[motor_id]
        self.servo.write_u16(
            motor_id,
            self.servo.ADDR_GOAL_SPEED,
            signed_to_raw(corrected),
        )

    def drive(self, left: int, right: int, force: bool = False) -> None:
        speeds = (int(left), int(right))
        if not force and speeds == self.last_speeds:
            return
        self.set_speed(1, speeds[0])
        self.set_speed(3, speeds[0])
        self.set_speed(2, speeds[1])
        self.set_speed(4, speeds[1])
        self.last_speeds = speeds

    def stop(self) -> None:
        self.drive(0, 0, force=True)

    def close(self) -> None:
        if not self.opened:
            return
        try:
            self.stop()
        finally:
            for motor_id in MOTOR_IDS:
                try:
                    self.servo.torque(motor_id, False)
                except Exception:
                    pass
            self.servo.close()
            self.opened = False


def steering(left_cm: float | None, right_cm: float | None) -> tuple[int, int, str]:
    if left_cm is None or right_cm is None:
        return 0, 0, "STOP sensor invalid"

    left_active = left_cm < CORRECTION_RANGE_CM
    right_active = right_cm < CORRECTION_RANGE_CM

    # 両側とも10 cm以上なら、側面センサーによる補正は行わない。
    if not left_active and not right_active:
        return POWER, POWER, "FORWARD no side correction"

    # 両側が5 cm未満なら逃げる方向を決められないため停止する。
    if left_cm < MIN_CLEARANCE_CM and right_cm < MIN_CLEARANCE_CM:
        return 0, 0, "STOP both sides close"

    # 片側だけが10 cm以内なら、その壁から5 cmを目標に追従する。
    if left_active and not right_active:
        error = left_cm - MIN_CLEARANCE_CM
        if abs(error) <= CENTER_DEADBAND_CM:
            return POWER, POWER, "FORWARD follow left wall"
        steer_left = error > 0  # 遠いなら左壁へ寄る。近いなら右へ離れる。
        state = "CORRECT LEFT toward left wall" if steer_left else "CORRECT RIGHT away from left wall"
    elif right_active and not left_active:
        error = right_cm - MIN_CLEARANCE_CM
        if abs(error) <= CENTER_DEADBAND_CM:
            return POWER, POWER, "FORWARD follow right wall"
        steer_left = error < 0  # 近いなら左へ離れる。遠いなら右壁へ寄る。
        state = "CORRECT LEFT away from right wall" if steer_left else "CORRECT RIGHT toward right wall"
    else:
        # 両側が10 cm以内なら、左右距離が等しくなるよう中央へ寄せる。
        error = left_cm - right_cm
        if abs(error) <= CENTER_DEADBAND_CM:
            return POWER, POWER, "FORWARD centered"
        steer_left = error > 0  # 右が近いなら左へ補正。
        state = "CORRECT LEFT" if steer_left else "CORRECT RIGHT"

    amount = min(MAX_CORRECTION, abs(error) * CORRECTION_PER_CM)
    inside_speed = round(POWER * (1.0 - 2.0 * amount))
    if steer_left:
        return POWER, inside_speed, state
    return inside_speed, POWER, state


def main() -> int:
    print("side_guidance_test.py started", flush=True)
    print("最初は車輪を浮かせ、左右の補正方向を確認してください。", flush=True)
    sensors = SideSensors()
    drive = Drive()
    try:
        sensors.start()
        deadline = time.monotonic() + 15.0
        left = right = None
        last_print = 0.0
        while time.monotonic() < deadline:
            left, right = sensors.read()
            if left is not None and right is not None:
                break
            now = time.monotonic()
            if now - last_print >= 0.5:
                left_text = "---" if left is None else f"{left:.1f}"
                right_text = "---" if right is None else f"{right:.1f}"
                print(
                    f"waiting: LEFT={left_text} cm, RIGHT={right_text} cm",
                    flush=True,
                )
                last_print = now
            time.sleep(0.05)
        if left is None or right is None:
            raise RuntimeError(
                "左右両方の測距値を取得できません。"
                "各センサー正面の5～50 cmに板を置いてください"
            )

        drive.start()
        input(
            f"LEFT={left:.1f} cm, RIGHT={right:.1f} cm。"
            "Enterで最大1秒動かします: "
        )
        started = time.monotonic()
        last_displayed = None
        while time.monotonic() - started < MAX_RUN_SECONDS:
            left, right = sensors.read()
            left_speed, right_speed, state = steering(left, right)
            drive.drive(left_speed, right_speed)
            left_text = "---" if left is None else f"{left:.1f}"
            right_text = "---" if right is None else f"{right:.1f}"
            displayed = (
                None if left is None else round(left, 1),
                None if right is None else round(right, 1),
                state,
                left_speed,
                right_speed,
            )
            if displayed != last_displayed:
                print(
                    f"L={left_text} R={right_text} | {state} | "
                    f"motor=({left_speed},{right_speed})",
                    flush=True,
                )
                last_displayed = displayed
            if left_speed == 0 and right_speed == 0:
                break
            time.sleep(0.05)
        return 0
    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}", flush=True)
        return 1
    finally:
        try:
            drive.close()
        except Exception:
            pass
        sensors.close()
        print("Finished", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
