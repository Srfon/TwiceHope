import smbus2
import time
import math

# -------------------------
# Madgwick フィルタ
# -------------------------
class MadgwickAHRS:
    def __init__(self, beta=0.1):
        self.beta = beta
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def updateIMU(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax /= norm
        ay /= norm
        az /= norm

        f1 = 2*(q1*q3 - q0*q2) - ax
        f2 = 2*(q0*q1 + q2*q3) - ay
        f3 = 2*(0.5 - q1*q1 - q2*q2) - az

        J_11 = -2*q2
        J_12 = 2*q3
        J_13 = -2*q0
        J_14 = 2*q1

        J_21 = 2*q1
        J_22 = 2*q0
        J_23 = 2*q3
        J_24 = 2*q2

        J_31 = 0
        J_32 = -4*q1
        J_33 = -4*q2
        J_34 = 0

        s0 = J_11*f1 + J_21*f2 + J_31*f3
        s1 = J_12*f1 + J_22*f2 + J_32*f3
        s2 = J_13*f1 + J_23*f2 + J_33*f3
        s3 = J_14*f1 + J_24*f2 + J_34*f3

        norm_s = math.sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        s0 /= norm_s
        s1 /= norm_s
        s2 /= norm_s
        s3 /= norm_s

        gx = math.radians(gx)
        gy = math.radians(gy)
        gz = math.radians(gz)

        qDot0 = 0.5*(-q1*gx - q2*gy - q3*gz) - self.beta*s0
        qDot1 = 0.5*( q0*gx + q2*gz - q3*gy) - self.beta*s1
        qDot2 = 0.5*( q0*gy - q1*gz + q3*gx) - self.beta*s2
        qDot3 = 0.5*( q0*gz + q1*gy - q2*gx) - self.beta*s3

        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt

        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        self.q0, self.q1, self.q2, self.q3 = q0/norm_q, q1/norm_q, q2/norm_q, q3/norm_q

    def getEuler(self):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        roll  = math.degrees(math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)))
        pitch = math.degrees(math.asin(2*(q0*q2 - q3*q1)))
        yaw   = math.degrees(math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)))

        return roll, pitch, yaw

# -------------------------
# IMU 初期化
# -------------------------
bus = smbus2.SMBus(1)
DEV_ADDR = 0x6B

def write_reg(reg, val):
    bus.write_byte_data(DEV_ADDR, reg, val)

def read_bytes(reg, length):
    try:
        return bus.read_i2c_block_data(DEV_ADDR, reg, length)
    except:
        return None

write_reg(0x12, 0x01)
time.sleep(0.05)
write_reg(0x10, 0b00001001)
write_reg(0x11, 0b00001001)
write_reg(0x15, 0b00000100)
write_reg(0x17, 0b10000010)

GYRO_SENS = 0.07
ACC_SENS  = 0.000244

# -------------------------
# ジャイロバイアス測定
# -------------------------
yaw = 0.0
DRIFT_TIME = 5.0
start_t = time.time()
last_t  = start_t

first_yaw = None
last_yaw  = None
first_t   = None
last_t_for_bias = None

print("[DRIFT] start")

while True:
    now = time.time()
    dt = now - last_t
    last_t = now

    buf = read_bytes(0x22, 12)
    if buf is None:
        continue

    vals = [int.from_bytes(buf[i*2:i*2+2], 'little', signed=True) for i in range(6)]
    gz_raw = vals[2]
    gz = gz_raw * GYRO_SENS

    yaw += gz * dt

    if first_yaw is None:
        first_yaw = yaw
        first_t   = now

    last_yaw = yaw
    last_t_for_bias = now

    if now - start_t >= DRIFT_TIME:
        break

delta_yaw = last_yaw - first_yaw
delta_t   = last_t_for_bias - first_t
gyro_bias = delta_yaw / delta_t

print(f"[DRIFT] gyro_bias={gyro_bias:.6f} dps")

# -------------------------
# Madgwick フィルタ開始
# -------------------------
filt = MadgwickAHRS(beta=0.95)

yaw = 0.0
last_t = time.time()

# ★ print 間隔管理
last_print = time.time()
PRINT_INTERVAL = 0.5

while True:
    now = time.time()
    dt = now - last_t
    last_t = now

    buf = read_bytes(0x22, 12)
    if buf is None:
        continue

    gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = [
        int.from_bytes(buf[i*2:i*2+2], 'little', signed=True) for i in range(6)
    ]

    gx = gx_raw * GYRO_SENS
    gy = gy_raw * GYRO_SENS
    gz = gz_raw * GYRO_SENS - gyro_bias   # ★ バイアス補正

    ax = ax_raw * ACC_SENS
    ay = ay_raw * ACC_SENS
    az = az_raw * ACC_SENS

    # Madgwick 更新
    filt.updateIMU(gx, gy, gz, ax, ay, az, dt)

    roll, pitch, yaw = filt.getEuler()

    # ★ 0.5秒ごとに print
    if now - last_print >= PRINT_INTERVAL:
        print(f"Madgwick roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")
        print("----")
        last_print = now

    time.sleep(0.001)
