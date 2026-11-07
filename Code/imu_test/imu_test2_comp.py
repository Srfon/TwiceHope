import smbus2
import time
import math

bus = smbus2.SMBus(1)
DEV_ADDR = 0x6B

def write_reg(reg, val):
    bus.write_byte_data(DEV_ADDR, reg, val)

def read_bytes(reg, length):
    try:
        return bus.read_i2c_block_data(DEV_ADDR, reg, length)
    except:
        return None

# -------------------------
# IMU 初期化（秋月準拠）
# -------------------------
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
# Complementary Filter 開始
# -------------------------
alpha = 0.97   # ★ ジャイロ重み（推奨：0.95〜0.99）

roll = 0.0
pitch = 0.0
yaw = 0.0

last_t = time.time()
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
    gz = gz_raw * GYRO_SENS - gyro_bias

    ax = ax_raw * ACC_SENS
    ay = ay_raw * ACC_SENS
    az = az_raw * ACC_SENS

    # 加速度から roll/pitch を計算
    acc_roll  = math.degrees(math.atan2(ay, az))
    acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

    # Complementary Filter
    roll  = alpha * (roll  + gx * dt) + (1 - alpha) * acc_roll
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * acc_pitch

    # yaw はジャイロ積分のみ（地磁気なし）
    yaw += gz * dt

    # print を 0.5秒間隔で実行
    if now - last_print >= PRINT_INTERVAL:
        print(f"CF roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")
        print("----")
        last_print = now

    time.sleep(0.001)
