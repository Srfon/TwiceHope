import smbus2
import time

bus = smbus2.SMBus(1)
ADDR = 0x6B

# Registers
REG_FUNC_CFG_ACCESS = 0x01
REG_FIFO_CTRL1      = 0x07
REG_FIFO_CTRL2      = 0x08
REG_FIFO_CTRL3      = 0x09
REG_FIFO_CTRL4      = 0x0A
REG_CTRL1_XL        = 0x10
REG_CTRL2_G         = 0x11
REG_CTRL3_C         = 0x12
REG_FIFO_STATUS1    = 0x3A
REG_FIFO_STATUS2    = 0x3B
REG_FIFO_DATA_OUT   = 0x40

def write_reg(reg, val):
    bus.write_byte_data(ADDR, reg, val)

def read_bytes(reg, length):
    try:
        return bus.read_i2c_block_data(ADDR, reg, length)
    except Exception as e:
        print("I2C ERROR:", e)
        return None

# -------------------------
# Correct FIFO initialization
# -------------------------

print("RESET...")
write_reg(REG_CTRL3_C, 0x01)   # SW_RESET
time.sleep(0.05)

print("BDU ON...")
write_reg(REG_CTRL3_C, 0x40)   # BDU=1

print("Enable accel + gyro...")
write_reg(REG_CTRL1_XL, 0b01100000)  # Accel 416Hz
write_reg(REG_CTRL2_G,  0b01100000)  # Gyro 416Hz

print("FIFO watermark...")
write_reg(REG_FIFO_CTRL1, 20)
write_reg(REG_FIFO_CTRL2, 0)

print("FIFO batching...")
write_reg(REG_FIFO_CTRL3, 0b00000101)  # gyro + accel

print("FIFO mode...")
write_reg(REG_FIFO_CTRL4, 0b00000110)  # continuous mode

time.sleep(0.1)

# -------------------------
# RAW FIFO dump
# -------------------------

print("Start RAW FIFO dump...")

while True:
    status = read_bytes(REG_FIFO_STATUS1, 2)
    if status is None:
        continue

    fifo_level = status[0] | (status[1] << 8)

    if fifo_level > 0:
        print(f"\nFIFO level={fifo_level}")

        for i in range(fifo_level):
            raw = read_bytes(REG_FIFO_DATA_OUT, 7)
            if raw is None:
                print("READ ERROR at sample", i)
                break

            hexraw = " ".join(f"{b:02X}" for b in raw)
            print(f"[{i:03d}] RAW: {hexraw}")

    time.sleep(0.05)
