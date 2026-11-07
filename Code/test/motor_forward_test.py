#!/usr/bin/env python3
"""STS3215 4輪を0.5秒だけ前進させる独立テスト。"""

import time

from sts3215 import STS3215

PORT = "/dev/ttyUSB0"
BAUDRATE = 1_000_000
MOTOR_IDS = (1, 2, 3, 4)
MOTOR_DIRECTION = {1: 1, 2: -1, 3: 1, 4: -1}
POWER = 300
DURATION_SECONDS = 0.5


def signed_to_raw(value: int) -> int:
    magnitude = max(0, min(4095, abs(int(value))))
    return magnitude | 0x8000 if value < 0 else magnitude


def set_speed(servo: STS3215, motor_id: int, speed: int) -> None:
    corrected = speed * MOTOR_DIRECTION[motor_id]
    servo.write_u16(motor_id, servo.ADDR_GOAL_SPEED, signed_to_raw(corrected))


def stop_all(servo: STS3215) -> None:
    for motor_id in MOTOR_IDS:
        set_speed(servo, motor_id, 0)


def main() -> None:
    print("motor_forward_test.py started", flush=True)
    print("車輪を浮かせるか、ロボットの前方を空けてください。", flush=True)
    input("Enterを押すと0.5秒だけ前進します: ")

    servo = STS3215(port=PORT, baudrate=BAUDRATE, timeout=0.1)
    servo.open()
    try:
        for motor_id in MOTOR_IDS:
            print(f"PING ID={motor_id}", flush=True)
            if not servo.ping(motor_id):
                raise RuntimeError(f"Servo ID={motor_id} not found")

        for motor_id in MOTOR_IDS:
            servo.torque(motor_id, False)
            servo.set_mode(motor_id, STS3215.MODE_VELOCITY)
            servo.torque(motor_id, True)

        print(f"FORWARD power={POWER}", flush=True)
        for motor_id in MOTOR_IDS:
            set_speed(servo, motor_id, POWER)
        time.sleep(DURATION_SECONDS)

    finally:
        print("STOP", flush=True)
        try:
            stop_all(servo)
        finally:
            for motor_id in MOTOR_IDS:
                try:
                    servo.torque(motor_id, False)
                except Exception:
                    pass
            servo.close()


if __name__ == "__main__":
    main()