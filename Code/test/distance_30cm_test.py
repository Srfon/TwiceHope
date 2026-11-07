#!/usr/bin/env python3
"""STS3215位置フィードバックで300 mm前進し、VL53L1Xでも非常停止する。"""

import time

try:
    from robot_hardware import (
        EMERGENCY_DISTANCE_CM,
        FrontDistanceSensor,
        MotionCommand,
        SafeMotionBridge,
        WheelOdometry,
        open_default_drive,
    )
except ModuleNotFoundError as error:
    if error.name != "robot_hardware":
        raise
    from robot_hardware import (
        EMERGENCY_DISTANCE_CM,
        FrontDistanceSensor,
        MotionCommand,
        SafeMotionBridge,
        WheelOdometry,
        open_default_drive,
    )

TARGET_DISTANCE_MM = 300.0
MIN_START_DISTANCE_CM = 35.0
MAX_RUN_SECONDS = 3.0
LOOP_SECONDS = 0.05


def wait_for_sensor(sensor: FrontDistanceSensor, timeout: float = 5.0) -> float:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = sensor.distance_cm()
        if value is not None:
            return value
        time.sleep(LOOP_SECONDS)
    raise RuntimeError(f"VL53L1X unavailable: {sensor.error or 'no measurement'}")


def main() -> int:
    print("distance_30cm_test.py started", flush=True)
    print("前方を40 cm以上空け、すぐ停止できる状態で実行してください。", flush=True)

    servo = None
    drive = None
    sensor = FrontDistanceSensor()
    bridge = None
    try:
        sensor.start()
        first_distance = wait_for_sensor(sensor)
        print(f"VL53L1X distance={first_distance:.1f} cm", flush=True)
        if first_distance < MIN_START_DISTANCE_CM:
            raise RuntimeError(
                f"前方距離が不足しています。{MIN_START_DISTANCE_CM:.0f} cm以上空けてください"
            )

        servo, drive = open_default_drive()
        bridge = SafeMotionBridge(drive, sensor)
        odometry = WheelOdometry(drive)
        odometry.reset()
        print("Encoder positions ready", flush=True)
        input(f"Enterを押すと最大{TARGET_DISTANCE_MM:.0f} mm前進します: ")

        started = time.monotonic()
        last_print = 0.0
        reason = "unknown"
        while True:
            now = time.monotonic()
            travelled_mm = odometry.update()
            front_cm = sensor.distance_cm()
            actual = bridge.apply(MotionCommand.FORWARD)

            if now - last_print >= 0.1:
                front_text = "---" if front_cm is None else f"{front_cm:.1f}"
                print(
                    f"travel={travelled_mm:+.1f} mm, "
                    f"front={front_text} cm, command={actual.value}",
                    flush=True,
                )
                last_print = now

            if actual == MotionCommand.STOP:
                reason = "VL53L1X safety stop"
                break
            if abs(travelled_mm) >= TARGET_DISTANCE_MM:
                bridge.emergency_stop()
                reason = "300 mm odometry stop"
                break
            if now - started >= MAX_RUN_SECONDS:
                bridge.emergency_stop()
                reason = "timeout safety stop"
                break
            time.sleep(LOOP_SECONDS)

        final_mm = odometry.update()
        print(f"STOP: {reason}; final travel={final_mm:+.1f} mm", flush=True)
        if final_mm < 0:
            print("WARNING: encoder sign is reversed", flush=True)
        return 0

    except Exception as error:
        print(f"ERROR: {type(error).__name__}: {error}", flush=True)
        return 1
    finally:
        if bridge is not None:
            bridge.emergency_stop()
        elif drive is not None:
            drive.stop(force=True)
        sensor.stop()
        if drive is not None:
            drive.shutdown()
        if servo is not None:
            servo.close()
        print("Finished", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
