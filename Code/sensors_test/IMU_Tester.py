import time
import math
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_LINEAR_ACCELERATION,
)

def quaternion_to_euler_bno085(i, j, k, r):
    # BNO085 �� �ʏ�E��n�֎��ϊ�
    # BNO085: X=�E, Y=�O, Z=��
    # �ʏ�:   X=�O, Y=��, Z=��
    # �� X'=Y, Y'=-X, Z'=Z
    x = j
    y = -i
    z = k
    w = r

    # Roll (X��)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y��)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (Z��)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(-roll), math.degrees(pitch), math.degrees(-yaw)

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def get_euler_dict(qi, qj, qk, qr):
    roll, pitch, yaw = quaternion_to_euler_bno085(qi, qj, qk, qr)
    return {
        "roll": normalize_angle(roll),
        "pitch": normalize_angle(pitch),
        "yaw": normalize_angle(yaw)
    }

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)

print("BNO085 Ready")

while True:
    qi, qj, qk, qr = bno.quaternion

    euler = get_euler_dict(qi, qj, qk, qr)

    ax, ay, az = bno.linear_acceleration

    print(
        f"Roll={euler['roll']:6.2f}��, "
        f"Pitch={euler['pitch']:6.2f}��, "
        f"Yaw={euler['yaw']:6.2f}�� | "
        f"Ax={ax:6.3f}, Ay={ay:6.3f}, Az={az:6.3f}"
    )

    time.sleep(0.1)