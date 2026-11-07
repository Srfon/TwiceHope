from smbus2 import SMBus
import time

bus = SMBus(1)

ADDR = 0x29

# Power ON
bus.write_byte_data(ADDR, 0x80, 0x03)

time.sleep(0.1)

for reg in [0x94,0x96,0x98,0x9A]:
    low = bus.read_byte_data(ADDR, reg)
    high = bus.read_byte_data(ADDR, reg+1)
    value = (high << 8) | low
    print(hex(reg), value)