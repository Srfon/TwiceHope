#!/usr/bin/env python3

from sts3215 import STS3215
import time

PORT = "/dev/ttyUSB0"
BAUDRATE = 1_000_000


servo = STS3215(
    port=PORT,
    baudrate=BAUDRATE
)


try:

    print("========================================")
    print(" STS3215 TEST")
    print("========================================")
    print(f"PORT : {PORT}")
    print(f"RATE : {BAUDRATE}")
    print()

    servo.open()

    # ------------------------------------------------------------
    # Scan
    # ------------------------------------------------------------

    print("[SCAN]")

    ids = servo.scan(0, 20)

    print("Detected:", ids)
    print()

    # ------------------------------------------------------------
    # Read
    # ------------------------------------------------------------

    for servo_id in ids:

        print(
            f"--- SERVO ID={servo_id} ---"
        )

        position = servo.get_position(
            servo_id
        )

        voltage = servo.get_voltage(
            servo_id
        )

        temperature = servo.get_temperature(
            servo_id
        )

        moving = servo.is_moving(
            servo_id
        )

        print(
            f"Position    : {position}"
        )

        print(
            f"Angle       : "
            f"{position * 360.0 / 4096.0:.2f} deg"
        )

        print(
            f"Voltage     : {voltage:.1f} V"
        )

        print(
            f"Temperature : {temperature} C"
        )

        print(
            f"Moving      : {moving}"
        )

        print()

        servo.set_speed(1,400)
        time.sleep(1)

        servo.set_speed(1,0)


finally:

    servo.close()

    print("Serial closed")