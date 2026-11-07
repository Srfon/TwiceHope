import math

def quaternion_to_euler(i, j, k, r):
    # Roll
    sinr_cosp = 2 * (r * i + j * k)
    cosr_cosp = 1 - 2 * (i * i + j * j)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2 * (r * j - k * i)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw
    siny_cosp = 2 * (r * k + i * j)
    cosy_cosp = 1 - 2 * (j * j + k * k)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    return roll, pitch, yaw


def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_euler_dict(qi, qj, qk, qr):
    roll, pitch, yaw = quaternion_to_euler(qi, qj, qk, qr)

    return {
        "roll": normalize_angle(roll),
        "pitch": normalize_angle(pitch),
        "yaw": normalize_angle(yaw)
    }
