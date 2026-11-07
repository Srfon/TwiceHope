#!/usr/bin/env python3

import time

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
    print(" STS3215 MOVE TEST")
    print("========================================")

    # ------------------------------------------------------------
    # Ping
    # ------------------------------------------------------------

    if not servo.ping(SERVO_ID):
        raise RuntimeError(
            "Servo not found"
        )

    # ------------------------------------------------------------
    # Current position
    # ------------------------------------------------------------

    current = servo.get_position(
        SERVO_ID
    )

    print(
        f"Current position : {current}"
    )

    # ------------------------------------------------------------
    # Torque ON
    # ------------------------------------------------------------

    print("Torque ON")

    servo.torque(
        SERVO_ID,
        True
    )

    time.sleep(0.2)

    # ------------------------------------------------------------
    # Speed
    # ------------------------------------------------------------

    print("Set speed = 500")

    servo.set_speed(
        SERVO_ID,
        500
    )

    # ------------------------------------------------------------
    # Small movement
    # ------------------------------------------------------------

    if current > 2048:
        target = current - 700
    else:
        target = current + 700

    target = max(
        0,
        min(4095, target)
    )

    print(
        f"Move {current} -> {target}"
    )

    servo.set_position(
        SERVO_ID,
        target
    )

    # ------------------------------------------------------------
    # Monitor
    # ------------------------------------------------------------

    for _ in range(30):

        position = servo.get_position(
            SERVO_ID
        )

        moving = servo.is_moving(
            SERVO_ID
        )

        print(
            f"position={position:4d} "
            f"moving={moving}"
        )

        if not moving:
            break

        time.sleep(0.1)

    print("Move finished")

finally:

    # ------------------------------------------------------------
    # Torque OFF
    # ------------------------------------------------------------

    try:

        servo.torque(
            SERVO_ID,
            False
        )

        print("Torque OFF")

    except Exception as e:

        print(
            f"Torque OFF failed: {e}"
        )

    servo.close()

    print("Serial closed")