import time
from servo.sts3032_4 import STS3032Bus

if __name__ == "__main__":
    motor = STS3032Bus("/dev/ttyUSB0")
    motor.enable()
    motor.zero_all()

    motor.move({
        1: 90,
        2: -90,
        3: 90,
        4: 90,
    })
    for _ in range(300):
        motor.update()
        time.sleep(0.03)
    motor.off()
    motor.close()
