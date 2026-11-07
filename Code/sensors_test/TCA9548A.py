from smbus2 import SMBus
import time

I2C_BUS = 1

TCA_ADDR = 0x70
TCS_ADDR = 0x29

bus = SMBus(I2C_BUS)

def select_channel(ch):
    bus.write_byte(TCA_ADDR, 1 << ch)

def tcs_init():
    # ENABLE���W�X�^
    bus.write_byte_data(TCS_ADDR, 0x80, 0x03)
    time.sleep(0.01)

def read_rgbc():
    c = bus.read_word_data(TCS_ADDR, 0x94)
    r = bus.read_word_data(TCS_ADDR, 0x96)
    g = bus.read_word_data(TCS_ADDR, 0x98)
    b = bus.read_word_data(TCS_ADDR, 0x9A)
    return c, r, g, b

# CH0������
select_channel(0)
tcs_init()

# CH1������
select_channel(1)
tcs_init()

print("Start")

while True:

    select_channel(0)
    c0, r0, g0, b0 = read_rgbc()

    select_channel(1)
    c1, r1, g1, b1 = read_rgbc()

    print(
        f"CH0 C:{c0:5d} R:{r0:5d} G:{g0:5d} B:{b0:5d} | "
        f"CH1 C:{c1:5d} R:{r1:5d} G:{g1:5d} B:{b1:5d}"
    )

    time.sleep(0.2)   # 5Hz