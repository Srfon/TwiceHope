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
# IMU 初期化
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
# 初期ジャイロバイアス測定
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
# Complementary Filter（動的 α + 自動キャリブレーション）
# -------------------------
alpha_normal = 0.97
alpha = alpha_normal

roll = 0.0
pitch = 0.0
yaw = 0.0

last_t = time.time()
last_print = time.time()
PRINT_INTERVAL = 0.5

# 振動検出用
G_LIMIT = 1.3      # ★ 指定 g（例：1.3g を超えたら振動と判定）
G_STABLE = 1.05    # ★ 収まった判定（1.05g 以下）

vibration_mode = False
recalib_start = None

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

    acc_norm = math.sqrt(ax*ax + ay*ay + az*az)

    # -------------------------
    # 振動検出 → α=1.0（補正停止）
    # -------------------------
    if acc_norm > G_LIMIT:
        vibration_mode = True
        alpha = 1.0  # 補正停止
        recalib_start = None  # キャリブレーション準備
    else:
        # -------------------------
        # 振動が収まった瞬間 → キャリブレーション開始
        # -------------------------
        if vibration_mode and acc_norm < G_STABLE:
            vibration_mode = False
            recalib_start = now
            print("[CALIB] vibration ended → recalibration start")

        # -------------------------
        # キャリブレーション中（0.5秒間）
        # -------------------------
        if recalib_start is not None:
            if now - recalib_start < 0.5:
                # roll/pitch を加速度から再同期
                roll  = math.degrees(math.atan2(ay, az))
                pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
                # yaw はジャイロバイアス補正で自然回帰
                alpha = 0.0  # 加速度を強く信じる
            else:
                recalib_start = None
                alpha = alpha_normal
                print("[CALIB] complete → normal mode")

        # 通常モード
        if recalib_start is None:
            alpha = alpha_normal

    # -------------------------
    # Complementary Filter
    # -------------------------
    acc_roll  = math.degrees(math.atan2(ay, az))
    acc_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

    roll  = alpha * (roll  + gx * dt) + (1 - alpha) * acc_roll
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * acc_pitch

    yaw += gz * dt

    # -------------------------
    # print（0.5秒間隔）
    # -------------------------
    if now - last_print >= PRINT_INTERVAL:
        print(f"CF roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}  acc_norm={acc_norm:.2f}  alpha={alpha:.2f}")
        print("----")
        last_print = now

    time.sleep(0.001)
