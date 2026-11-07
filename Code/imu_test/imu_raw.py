import smbus2
import time

bus = smbus2.SMBus(1)

DEV_ADDR = 0x6B


# ============================================================
# I2C
# ============================================================

def write_reg(reg, val):
    bus.write_byte_data(DEV_ADDR, reg, val)


def read_bytes(reg, length):
    try:
        return bus.read_i2c_block_data(DEV_ADDR, reg, length)
    except Exception:
        return None


# ============================================================
# IMU 初期化
# ============================================================

write_reg(0x12, 0x01)
time.sleep(0.05)

write_reg(0x10, 0b00001001)
write_reg(0x11, 0b00001001)
write_reg(0x15, 0b00000100)
write_reg(0x17, 0b10000010)


# ============================================================
# Sensitivity
# ============================================================

GYRO_SENS = 0.07


# ============================================================
# Calibration
# ============================================================

CALIB_TIME = 5.0

print()
print("========================================")
print(" GYRO DRIFT CALIBRATION")
print("========================================")
print()
print("Keep the IMU completely still.")
print(f"Calibration time : {CALIB_TIME:.1f} sec")
print()


# ============================================================
# キャリブレーション
#
# 時間軸はここから開始する
#
# t = 0
#     ↓
#     5秒
#     ↓
# 通常動作
#
# この時間軸を通常動作でも継続して使用する
# ============================================================

calib_start = time.time()
last_t = calib_start

times = []

gx_samples = []
gy_samples = []
gz_samples = []


while True:

    now = time.time()

    dt = now - last_t
    last_t = now

    t = now - calib_start

    buf = read_bytes(0x22, 12)

    if buf is None:
        continue

    gx_raw, gy_raw, gz_raw, _, _, _ = [
        int.from_bytes(
            buf[i * 2:i * 2 + 2],
            "little",
            signed=True
        )
        for i in range(6)
    ]

    gx = gx_raw * GYRO_SENS
    gy = gy_raw * GYRO_SENS
    gz = gz_raw * GYRO_SENS

    times.append(t)

    gx_samples.append(gx)
    gy_samples.append(gy)
    gz_samples.append(gz)

    if t >= CALIB_TIME:
        break

    time.sleep(0.001)


# ============================================================
# 一次回帰
#
# G(t) = A + k*t
#
# A = 初期オフセット
# k = ドリフト速度
# ============================================================

def linear_regression(x, y):

    n = len(x)

    if n < 2:
        raise RuntimeError("Not enough calibration samples")

    mean_x = sum(x) / n
    mean_y = sum(y) / n

    numerator = 0.0
    denominator = 0.0

    for xi, yi in zip(x, y):

        dx = xi - mean_x
        dy = yi - mean_y

        numerator += dx * dy
        denominator += dx * dx

    if denominator == 0.0:
        raise RuntimeError("Invalid calibration data")

    k = numerator / denominator

    A = mean_y - k * mean_x

    return A, k


# ============================================================
# A + k*t
# ============================================================

offset_A_roll, drift_roll = linear_regression(
    times,
    gx_samples
)

offset_A_pitch, drift_pitch = linear_regression(
    times,
    gy_samples
)

offset_A_yaw, drift_yaw = linear_regression(
    times,
    gz_samples
)


# ============================================================
# キャリブレーション結果
# ============================================================

print()
print("[CALIB] complete")
print()

print("Offset A")
print(
    f"  Roll  = {offset_A_roll:+.6f} dps"
)
print(
    f"  Pitch = {offset_A_pitch:+.6f} dps"
)
print(
    f"  Yaw   = {offset_A_yaw:+.6f} dps"
)

print()

print("Drift rate")
print(
    f"  Roll  = {drift_roll:+.6f} dps/s"
)
print(
    f"  Pitch = {drift_pitch:+.6f} dps/s"
)
print(
    f"  Yaw   = {drift_yaw:+.6f} dps/s"
)

print()

# ============================================================
# キャリブレーション終了時点のB
# ============================================================

offset_B_roll_calib = drift_roll * CALIB_TIME
offset_B_pitch_calib = drift_pitch * CALIB_TIME
offset_B_yaw_calib = drift_yaw * CALIB_TIME

print("Offset B at calibration end")

print(
    f"  Roll  = {offset_B_roll_calib:+.6f} dps"
)

print(
    f"  Pitch = {offset_B_pitch_calib:+.6f} dps"
)

print(
    f"  Yaw   = {offset_B_yaw_calib:+.6f} dps"
)

print()
print("========================================")
print()


# ============================================================
# 通常動作
#
# runtime = 通常動作開始からの時間
#
# absolute_time =
#     キャリブレーション時間 + runtime
#
# つまり時間軸をリセットしない
# ============================================================

roll = 0.0
pitch = 0.0
yaw = 0.0
raw_roll = 0.0
raw_pitch = 0.0
raw_yaw = 0.0

last_t = time.time()

runtime_start = last_t

last_print = last_t

PRINT_INTERVAL = 0.5


while True:

    now = time.time()

    dt = now - last_t
    last_t = now

    # --------------------------------------------------------
    # 通常動作開始からの時間
    # --------------------------------------------------------

    runtime = now - runtime_start

    # --------------------------------------------------------
    # キャリブレーション開始からの連続時間
    # --------------------------------------------------------

    absolute_time = CALIB_TIME + runtime

    buf = read_bytes(0x22, 12)

    if buf is None:
        continue

    gx_raw, gy_raw, gz_raw, _, _, _ = [
        int.from_bytes(
            buf[i * 2:i * 2 + 2],
            "little",
            signed=True
        )
        for i in range(6)
    ]

    # ========================================================
    # Raw gyro
    # ========================================================

    gx = gx_raw * GYRO_SENS
    gy = gy_raw * GYRO_SENS
    gz = gz_raw * GYRO_SENS


    # ========================================================
    # Offset B
    #
    # キャリブレーション終了後も
    # 同じ時間軸で継続
    # ========================================================

    offset_B_roll = drift_roll * absolute_time
    offset_B_pitch = drift_pitch * absolute_time
    offset_B_yaw = drift_yaw * absolute_time


    # ========================================================
    # A + B
    # ========================================================

    offset_roll = offset_A_roll + offset_B_roll
    offset_pitch = offset_A_pitch + offset_B_pitch
    offset_yaw = offset_A_yaw + offset_B_yaw


    # ========================================================
    # ジャイロ補正
    # ========================================================

    gx_corrected = gx - offset_roll
    gy_corrected = gy - offset_pitch
    gz_corrected = gz - offset_yaw

    raw_roll += gx *dt
    raw_pitch += gy *dt
    raw_yaw += gz *dt

    # ========================================================
    # 積分
    # ========================================================

    roll += gx_corrected * dt
    pitch += gy_corrected * dt
    yaw += gz_corrected * dt


    # ========================================================
    # 表示
    # ========================================================

    if now - last_print >= PRINT_INTERVAL:

        print(
            f"[RUN] "
            f"t={runtime:8.2f}s "
            f"ABS={absolute_time:8.2f}s"
        )
        print(
            f"      RAW "
            f"R={raw_yaw:+10.3f} "
            f"P={raw_pitch:+10.3f} "
            f"Y={raw_yaw:+10.3f}"
        )
        print(
            f"      ANGLE "
            f"R={roll:+10.3f} "
            f"P={pitch:+10.3f} "
            f"Y={yaw:+10.3f}"
        )

        print(
            f"      A "
            f"R={offset_A_roll:+9.5f} "
            f"P={offset_A_pitch:+9.5f} "
            f"Y={offset_A_yaw:+9.5f}"
        )

        print(
            f"      B "
            f"R={offset_B_roll:+9.5f} "
            f"P={offset_B_pitch:+9.5f} "
            f"Y={offset_B_yaw:+9.5f}"
        )

        print(
            f"      TOTAL "
            f"R={offset_roll:+9.5f} "
            f"P={offset_pitch:+9.5f} "
            f"Y={offset_yaw:+9.5f}"
        )

        print("----")

        last_print = now

    time.sleep(0.001)