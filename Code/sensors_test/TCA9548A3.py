from smbus2 import SMBus
import time
import board
import busio
import adafruit_vl53l1x

I2C_BUS = 1

TCA_ADDR = 0x70
TCS_ADDR = 0x29   # RGB�Z���T�[
VL_ADDR  = 0x29   # VL53L1X�i�����A�h���X����TCA�ŕ����j

bus = SMBus(I2C_BUS)

def safe_int_fmt(value, width=5):
    if value is None:
        return " " * width
    try:
        return f"{int(value):{width}d}"
    except:
        return " " * width

def select_channel(ch):
    bus.write_byte(TCA_ADDR, 1 << ch)

# -------------------------
# TCS34725 ������
# -------------------------
def tcs_init():
    bus.write_byte_data(TCS_ADDR, 0x80, 0x03)
    time.sleep(0.01)

def read_rgbc():
    c = bus.read_word_data(TCS_ADDR, 0x94)
    r = bus.read_word_data(TCS_ADDR, 0x96)
    g = bus.read_word_data(TCS_ADDR, 0x98)
    b = bus.read_word_data(TCS_ADDR, 0x9A)
    return c, r, g, b

# CH0 �� TCS34725
select_channel(2)
tcs_init()

# CH1 �� TCS34725
select_channel(3)
tcs_init()

# -------------------------
# VL53L1X �������iCH2 / CH3�j
# -------------------------
i2c = busio.I2C(board.SCL, board.SDA)

# CH2
select_channel(0)
vl53_ch0 = adafruit_vl53l1x.VL53L1X(i2c)
vl53_ch0.start_ranging()

# CH3
select_channel(1)
vl53_ch1 = adafruit_vl53l1x.VL53L1X(i2c)
vl53_ch1.start_ranging()

print("Start")

while True:
    # CH0 RGB
    select_channel(2)
    c0, r0, g0, b0 = read_rgbc()

    # CH1 RGB
    select_channel(3)
    c1, r1, g1, b1 = read_rgbc()

    # CH2 VL53L1X
    select_channel(0)
    d2 = vl53_ch0.distance

    # CH3 VL53L1X
    select_channel(1)
    d3 = vl53_ch1.distance

    print(
        f"CH0 C:{safe_int_fmt(c0)} R:{safe_int_fmt(r0)} G:{safe_int_fmt(g0)} B:{safe_int_fmt(b0)} | "
        f"CH1 C:{safe_int_fmt(c1)} R:{safe_int_fmt(r1)} G:{safe_int_fmt(g1)} B:{safe_int_fmt(b1)} | "
        f"CH2 Dist:{safe_int_fmt(d2)} mm | "
        f"CH3 Dist:{safe_int_fmt(d3)} mm"
    )   

    time.sleep(0.05)
