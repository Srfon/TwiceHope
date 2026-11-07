from servo.sts3032_3 import STSBus
import time

if __name__ == "__main__":

    bus = STSBus(
        "/dev/ttyUSB0"
    )

    bus.torque_on(1)
    bus.torque_on(2)
    bus.torque_on(3)
    bus.torque_on(4)

    bus.move_group({

        1: {
            "position": 1000,
            "speed": 1500
        },

        2: {
            "position": 2000,
            "speed": 1500
        },

        3: {
            "position": 1000,
            "speed": 1500
        },

        4: {
            "position": 1000,
            "speed": 1500
        }
    })

    time.sleep(3)

    #bus.move(
    #    sid=1,
    #    position=2048,
    #    speed=500
    #)
    bus.torque_off(1)
    bus.torque_off(2)
    bus.torque_off(3)
    bus.torque_off(4)

    bus.close()