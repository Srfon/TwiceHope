import smbus2
import time

bus = smbus2.SMBus(1)
DEV_ADDR = 0x6B

def write_reg(reg, val):
    bus.write_byte_data(DEV_ADDR, reg, val)

def read_bytes(reg, length):
    try:
        return bus.read_i2c_block_data(DEV_ADDR, reg, length)
    except:
        return None

# ============================================================
# IMU 初期化
# ============================================================
# ソフトリセット
write_reg(0x12, 0x01)
time.sleep(0.05)

# UI 加速度 104Hz / ±4g
write_reg(0x10, 0b01001001)

# UI ジャイロ 104Hz / ±500dps
write_reg(0x11, 0b01001001)

# EIS/OIS 無効化（UI のみ使用）
write_reg(0x15, 0x00)
write_reg(0x17, 0x00)


GYRO_SENS = 0.07
ACC_SENS  = 0.000244   # LSM6DSV16X UI accel sensitivity
# ============================================================
# キャリブレーション（静止時ドリフト測定）
# ============================================================
CALIB_TIME = 5.0
calib_roll = 0.0
calib_pitch = 0.0
calib_yaw = 0.0
get_count = 0

print("[CALIB] start... keep IMU perfectly still")

last_t = time.time()
calib_start = last_t

while True:
    now = time.time()
    dt = now - last_t
    last_t = now

    if now - calib_start >= CALIB_TIME:
        drift_roll  = calib_roll  / CALIB_TIME
        drift_pitch = calib_pitch / CALIB_TIME
        drift_yaw   = calib_yaw   / CALIB_TIME

        print("-----")
        print(f"[Count] {get_count}")
        print(f"[Drift] Roll  {drift_roll:8.4f} °/s")
        print(f"[Drift] Pitch {drift_pitch:8.4f} °/s")
        print(f"[Drift] Yaw   {drift_yaw:8.4f} °/s")
        print("Calib end...")
        print("-----")
        break

    buf = read_bytes(0x22, 12)
    if buf is None:
        continue

    gx_raw, gy_raw, gz_raw, _, _, _ = [
        int.from_bytes(buf[i*2:i*2+2], "little", signed=True)
        for i in range(6)
    ]

    calib_roll  += gx_raw * GYRO_SENS * dt
    calib_pitch += gy_raw * GYRO_SENS * dt
    calib_yaw   += gz_raw * GYRO_SENS * dt

    get_count += 1
    time.sleep(0.001)

# ============================================================
# メインループ（ドリフト補正付き積分）
# ============================================================
raw_roll = 0.0
raw_pitch = 0.0
raw_yaw = 0.0

loop_start = time.time()
last_t = loop_start
last_print = loop_start

PRINT_INTERVAL = 0.5

while True:
    now = time.time()
    dt = now - last_t
    last_t = now

    elapsed = now - loop_start

    # ============================================================
    # FIFO 読み出し（Accel + Gyro）
    # ============================================================

    buf = read_bytes(0x3E, 12)
    if buf is None:
        continue

    print("[FIFO RAW BIN]", " ".join(f"{b:02X}" for b in buf))

    # FIFO は Accel → Gyro の順で並ぶ
    ax_raw = int.from_bytes(buf[0:2],  "little", signed=True)
    ay_raw = int.from_bytes(buf[2:4],  "little", signed=True)
    az_raw = int.from_bytes(buf[4:6],  "little", signed=True)

    gx_raw = int.from_bytes(buf[6:8],  "little", signed=True)
    gy_raw = int.from_bytes(buf[8:10], "little", signed=True)
    gz_raw = int.from_bytes(buf[10:12],"little", signed=True)

    ax = ax_raw * ACC_SENS
    ay = ay_raw * ACC_SENS
    az = az_raw * ACC_SENS

    gx = gx_raw * GYRO_SENS
    gy = gy_raw * GYRO_SENS
    gz = gz_raw * GYRO_SENS

    raw_roll  += gx * dt
    raw_pitch += gy * dt
    raw_yaw   += gz * dt

    # 正しいドリフト補正（1つだけ）
    offseted_roll  = raw_roll  - drift_roll  * elapsed
    offseted_pitch = raw_pitch - drift_pitch * elapsed
    offseted_yaw   = raw_yaw   - drift_yaw   * elapsed

    if now - last_print >= PRINT_INTERVAL:
        print(f"[RUN] t={elapsed:8.2f}s")
        print(f"[RAW]      roll={raw_roll:8.2f} pitch={raw_pitch:8.2f} yaw={raw_yaw:8.2f}")
        print(f"[OFFSETED] roll={offseted_roll:8.2f} pitch={offseted_pitch:8.2f} yaw={offseted_yaw:8.2f}")
        print("----")
        last_print = now

    time.sleep(0.001)
