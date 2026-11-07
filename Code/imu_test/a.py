from smbus2 import SMBus
import time

I2C_BUS = 1
TCA_ADDR = 0x70   # TCA9548A のアドレス

bus = SMBus(I2C_BUS)

def select_channel(ch):
    """TCA9548A のチャネル選択"""
    bus.write_byte(TCA_ADDR, 1 << ch)
    time.sleep(0.005)

def scan_i2c_bus():
    """現在選択されている I2C バスをスキャン"""
    devices = []
    for addr in range(0x03, 0x78):
        try:
            bus.write_quick(addr)
            devices.append(addr)
        except:
            pass
    return devices

print("TCA9548A I2C Scan Start")

for ch in range(8):
    select_channel(ch)
    devs = scan_i2c_bus()

    print(f"CH{ch}: ", end="")
    if devs:
        print(", ".join([f"0x{d:02X}" for d in devs]))
    else:
        print("No device")
