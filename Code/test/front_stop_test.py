#!/usr/bin/env python3
"""前進中、VL53L1Xが5 cm以下を検出したら停止する実機テスト。"""

import time

try:
    from robot_hardware import (
        EMERGENCY_DISTANCE_CM,
        FrontDistanceSensor,
        MotionCommand,
        SafeMotionBridge,
        open_default_drive,
    )
except ModuleNotFoundError as error:
    if error.name != "robot_hardware":
        raise
    from ms200_viewer.robot_hardware import (
        EMERGENCY_DISTANCE_CM,
        FrontDistanceSensor,
        MotionCommand,
        SafeMotionBridge,
        open_default_drive,
    )

MAX_RUN_SECONDS = 1.0
LOOP_SECONDS = 0.05


def wait_for_sensor(sensor: FrontDistanceSensor, timeout: float = 5.0) -> float:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = sensor.distance_cm()
        if value is not None:
            return value
        time.sleep(LOOP_SECONDS)
    detail = sensor.error or "測距値が届きません"
    raise RuntimeError(f"VL53L1X unavailable: {detail}")


def main() -> int:
    print("front_stop_test.py started", flush=True)
    print("車輪を浮かせるか、前方を十分空けてください。", flush=True)

    servo = None
    drive = None
    sensor = FrontDistanceSensor()
    bridge = None
    try:
        print("VL53L1Xを開始します", flush=True)
        sensor.start()
        first_distance = wait_for_sensor(sensor)
        print(f"VL53L1X distance={first_distance:.1f} cm", flush=True)

        servo, drive = open_default_drive()
        bridge = SafeMotionBridge(drive, sensor)
        input(
            f"Enterを押すと前進します。{EMERGENCY_DISTANCE_CM:.1f} cm以下または"
            f"{MAX_RUN_SECONDS:.1f}秒で停止します: "
        )

        started = time.monotonic()
        last_print = 0.0
        while True:
            now = time.monotonic()
            distance = sensor.distance_cm()
            actual = bridge.apply(MotionCommand.FORWARD)

            if now - last_print >= 0.1:
                text = "---" if distance is None else f"{distance:.1f}"
                print(f"distance={text} cm, command={actual.value}", flush=True)
                last_print = now

            if actual == MotionCommand.STOP:
                print("VL53L1X safety stop", flush=True)
                break
            if now - started >= MAX_RUN_SECONDS:
                bridge.emergency_stop()
                print("Timeout safety stop", flush=True)
                break
            time.sleep(LOOP_SECONDS)
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
