#!/usr/bin/env python3

from sts3215 import STS3215


PORT = "/dev/ttyUSB0"
BAUDRATE = 1_000_000
SERVO_ID = 1


servo = STS3215(
    PORT,
    BAUDRATE
)

try:

    servo.open()

    print("========================================")
    print(" STS3215 STATUS")
    print("========================================")

    if not servo.ping(SERVO_ID):
        raise RuntimeError(
            f"Servo ID={SERVO_ID} not found"
        )

    status = servo.status(SERVO_ID)

    for key, value in status.items():

        print(
            f"{key:16s}: {value}"
        )

finally:

    servo.close()