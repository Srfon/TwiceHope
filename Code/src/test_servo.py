from servo.sts3032 import STS3032
import time

if __name__ == "__main__":

    servo = STS3032(
        "/dev/ttyUSB0",
        servo_id=1
    )


    print(
        servo.read_u16_debug(44)
    )

    print(
        servo.read_u16_debug(46)
    )
    print(
        "PING:",
        servo.ping()
    )

    print(
        "POS:",
        servo.get_position()
    )

    print(
        "TEMP:",
        servo.get_temperature()
    )

    print(
        "VOLT:",
        servo.get_voltage()
    )

    servo.move(1024)

    time.sleep(2)

    servo.move(3072)

    print("========================================")

    print("current =", servo.get_position())
    servo.move_speed(
        1024,
        300
    )

    time.sleep(2)

    servo.move_time(
        3072,
        1500
    )

    time.sleep(2)

    print("current =", servo.get_position())

    servo.close()