import time
from sts3215 import STS3215


PORT = "/dev/ttyUSB0"
SERVO_ID = 1
BAUDRATE = 1_000_000


def main():
    servo = STS3215(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=0.1,
    )

    servo.open()

    try:
        print("=" * 50)
        print(" STS3215 MODE / SPEED / TORQUE TEST")
        print("=" * 50)

        # --------------------------------------------------
        # 1. PING
        # --------------------------------------------------
        print("\n[1] PING")

        if not servo.ping(SERVO_ID):
            raise RuntimeError("Servo not found")

        print(f"Servo ID={SERVO_ID} : OK")

        # --------------------------------------------------
        # 2. 現在状態
        # --------------------------------------------------
        print("\n[2] CURRENT STATUS")

        mode = servo.get_mode(SERVO_ID)
        position = servo.get_position(SERVO_ID)
        speed = servo.get_speed(SERVO_ID)
        torque = servo.read_u8(SERVO_ID, servo.ADDR_TORQUE_ENABLE)
        torque_limit = servo.read_u16(
            SERVO_ID,
            servo.ADDR_TORQUE_LIMIT
        )

        print(f"Mode         : {mode}")
        print(f"Position     : {position}")
        print(f"Present Speed: {speed}")
        print(f"Torque       : {torque}")
        print(f"Torque Limit : {torque_limit}")

        # --------------------------------------------------
        # 3. トルクOFF
        # --------------------------------------------------
        print("\n[3] TORQUE OFF")

        servo.torque(SERVO_ID, False)

        print("Torque = OFF")
        time.sleep(0.5)

        # --------------------------------------------------
        # 4. 位置モード
        # --------------------------------------------------
        print("\n[4] SET POSITION MODE")

        servo.set_mode(
            SERVO_ID,
            servo.MODE_POSITION
        )

        mode = servo.get_mode(SERVO_ID)

        print(f"Mode = {mode}")

        # --------------------------------------------------
        # 5. トルクリミット設定
        # --------------------------------------------------
        print("\n[5] SET TORQUE LIMIT")

        torque_limit_value = 500

        servo.write_u16(
            SERVO_ID,
            servo.ADDR_TORQUE_LIMIT,
            torque_limit_value
        )

        read_limit = servo.read_u16(
            SERVO_ID,
            servo.ADDR_TORQUE_LIMIT
        )

        print(f"Torque Limit = {read_limit}")

        # --------------------------------------------------
        # 6. 速度設定
        # --------------------------------------------------
        print("\n[6] SET SPEED")

        speed_value = 500

        servo.set_speed(
            SERVO_ID,
            speed_value
        )

        read_speed = servo.read_u16(
            SERVO_ID,
            servo.ADDR_GOAL_SPEED
        )

        print(f"Goal Speed = {read_speed}")

        # --------------------------------------------------
        # 7. トルクON
        # --------------------------------------------------
        print("\n[7] TORQUE ON")

        servo.torque(
            SERVO_ID,
            True
        )

        print("Torque = ON")
        time.sleep(0.5)

        # --------------------------------------------------
        # 8. 小さく位置移動
        # --------------------------------------------------
        print("\n[8] SMALL POSITION MOVE")

        current = servo.get_position(SERVO_ID)

        print(f"Current Position = {current}")

        # 4095付近にいる場合は逆方向へ
        if current > 2000:
            target = current - 100
        else:
            target = current + 100

        target = max(0, min(4095, target))

        print(f"Target Position  = {target}")
        print(f"Speed            = {speed_value}")
        print(f"Torque Limit     = {torque_limit_value}")

        servo.set_position(
            SERVO_ID,
            target
        )

        # --------------------------------------------------
        # 9. 移動監視
        # --------------------------------------------------
        print("\n[9] MONITOR")

        for _ in range(30):

            position = servo.get_position(SERVO_ID)

            try:
                moving = servo.is_moving(SERVO_ID)
            except Exception:
                moving = -1

            print(
                f"Position={position:4d} "
                f"Moving={moving}"
            )

            if moving == 0:
                break

            time.sleep(0.1)

        # --------------------------------------------------
        # 10. トルクOFF
        # --------------------------------------------------
        print("\n[10] TORQUE OFF")

        servo.torque(
            SERVO_ID,
            False
        )

        print("Torque = OFF")

        # --------------------------------------------------
        # 11. 速度モードへ切替
        # --------------------------------------------------
        print("\n[11] SET VELOCITY MODE")

        servo.set_mode(
            SERVO_ID,
            servo.MODE_VELOCITY
        )

        mode = servo.get_mode(SERVO_ID)

        print(f"Mode = {mode}")

        # --------------------------------------------------
        # 12. 速度値確認
        # --------------------------------------------------
        print("\n[12] SPEED REGISTER")

        speed = servo.read_u16(
            SERVO_ID,
            servo.ADDR_GOAL_SPEED
        )

        print(f"Goal Speed = {speed}")

        # --------------------------------------------------
        # 終了
        # --------------------------------------------------
        print("\n" + "=" * 50)
        print(" TEST COMPLETE")
        print("=" * 50)

    except Exception as e:
        print()
        print("ERROR")
        print(type(e).__name__, e)

    finally:
        try:
            servo.torque(SERVO_ID, False)
        except Exception:
            pass

        servo.close()
        print("Serial closed")


if __name__ == "__main__":
    main()
