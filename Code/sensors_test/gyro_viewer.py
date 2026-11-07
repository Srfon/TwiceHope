import serial
import numpy as np
import cv2
import IMU_Caluc
import time
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

# I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)

# BNO085
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

print("BNO085 Ready")

WIDTH = 800
HEIGHT = 800
CX = WIDTH // 2
CY = HEIGHT // 2
SCALE = 260

cube = np.array([
    [-1, -1, -1],
    [ 1, -1, -1],
    [ 1,  1, -1],
    [-1,  1, -1],
    [-1, -1,  1],
    [ 1, -1,  1],
    [ 1,  1,  1],
    [-1,  1,  1],
], dtype=np.float32)

edges = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7)
]

yaw_offset = 0.0
pitch_offset = 0.0
roll_offset = 0.0

def rot_x(a):
    c = np.cos(a)
    s = np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def rot_y(a):
    c = np.cos(a)
    s = np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def rot_z(a):
    c = np.cos(a)
    s = np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def project(points):
    out = []
    for x,y,z in points:
        z += 4.0
        px = int(CX + (x / z) * SCALE)
        py = int(CY - (y / z) * SCALE)
        out.append((px,py))
    return out

cv2.namedWindow("IMU 3D")

while True:
    try:
        qi, qj, qk, qr = bno.quaternion

        angles = IMU_Caluc.get_euler_dict(qi, qj, qk, qr)

        yaw = angles["yaw"]
        pitch = angles["pitch"]
        roll = angles["roll"]

        yaw2 = yaw - yaw_offset
        pitch2 = pitch - pitch_offset
        roll2 = roll - roll_offset

        y = np.deg2rad(yaw2)
        p = np.deg2rad(-pitch2)
        r = np.deg2rad(roll2)

        Rx = rot_x(p)
        Ry = rot_y(r)
        Rz = rot_z(y)
        R = Rz @ Ry @ Rx

        rotated = cube @ R.T
        pts2d = project(rotated)

        img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        cv2.line(img, (CX,0), (CX,HEIGHT), (40,40,40), 1)
        cv2.line(img, (0,CY), (WIDTH,CY), (40,40,40), 1)

        for a,b in edges:
            cv2.line(img, pts2d[a], pts2d[b], (0,255,0), 2)

        cv2.putText(img, f"YAW   : {yaw2:7.2f}", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(img, f"PITCH : {pitch2:7.2f}", (20,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        cv2.putText(img, f"ROLL  : {roll2:7.2f}", (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

        cv2.imshow("IMU 3D", img)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            yaw_offset = yaw
            pitch_offset = pitch
            roll_offset = roll

        if key == 27:
            break

    except Exception as e:
        print("Parse error:", e)

cv2.destroyAllWindows()
