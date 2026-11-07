import smbus2
import time
import math
import traceback

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
# 初期化（秋月準拠）
# -------------------------
write_reg(0x12, 0x01)   # soft reset
time.sleep(0.05)

write_reg(0x10, 0b00001001)   # ACC
write_reg(0x11, 0b00001001)   # GYRO
write_reg(0x15, 0b00000100)   # enable
write_reg(0x17, 0b10000010)   # BDU + IF_INC

print("[INIT] complete")

# -------------------------
# スケール係数
# -------------------------
GYRO_SENS = 0.07
ACC_SENS  = 0.000244

# -------------------------
# yaw 初期化
# -------------------------
yaw = 0.0
yaw_offset = 0.0

# -------------------------
# ドリフト測定（起動直後の一回のみ）
# -------------------------
DRIFT_TIME = 5.0
start = time.time()
last_time = start

yaw = 0.0  # ★ 初期値は必ず 0°
drift_first = None
drift_last = None

print("[DRIFT] start")

# ログファイルを新規作成
log_file = open("log_yaw.txt", "w")

# -------------------------
# ドリフト測定ループ（起動直後のみ）
# -------------------------
while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    buf = read_bytes(0x22, 12)
    if buf is None:
        continue

    # raw 変換
    vals = []
    for i in range(6):
        vals.append(int.from_bytes(buf[i*2:i*2+2], 'little', signed=True))

    gz_raw = vals[2]
    gz = gz_raw * GYRO_SENS

    # yaw 積分
    yaw += gz * dt

    # ログ記録（timestamp, yaw）
    log_file.write(f"{now},{yaw:.6f}\n")

    # ドリフト測定中
    if now - start < DRIFT_TIME:
        if drift_first is None:
            drift_first = yaw
        drift_last = yaw
        print(f"[DRIFT] measuring... yaw={yaw:.2f}")
        continue

    # ドリフト測定終了（1回のみ）
    drift_amount = drift_last - drift_first
    yaw_offset = - drift_amount / DRIFT_TIME

    print(f"[DRIFT] drift_amount={drift_amount:.3f}, offset={yaw_offset:.3f}")
    print("[DRIFT] correction enabled")

    break

log_file.close()

# -------------------------
# メインループ（補正後 yaw のみ使用）
# -------------------------
last_time = time.time()

while True:
    now = time.time()
    dt = now - last_time
    last_time = now

    buf = read_bytes(0x22, 12)
    if buf is None:
        continue

    vals = []
    for i in range(6):
        vals.append(int.from_bytes(buf[i*2:i*2+2], 'little', signed=True))

    gx_raw, gy_raw, gz_raw, ax_raw, ay_raw, az_raw = vals

    gx = gx_raw * GYRO_SENS
    gy = gy_raw * GYRO_SENS
    gz = gz_raw * GYRO_SENS

    ax = ax_raw * ACC_SENS
    ay = ay_raw * ACC_SENS
    az = az_raw * ACC_SENS

    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))

    # yaw 積分 + オフセット補正
    yaw += gz * dt
    yaw_corrected = yaw + yaw_offset

    print(f"ACC[g] ax={ax:.3f} ay={ay:.3f} az={az:.3f}")
    print(f"GYRO[dps] gx={gx:.3f} gy={gy:.3f} gz={gz:.3f}")
    print(f"ANGLE roll={roll:.2f} pitch={pitch:.2f} yaw={yaw_corrected:.2f}")
    print("----")

    time.sleep(0.01)
