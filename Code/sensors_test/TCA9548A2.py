import smbus2
import time
import board
import busio
import adafruit_vl53l1x

# TCA9548A
bus = smbus2.SMBus(1)
TCA_ADDR = 0x70
CH2 = 0x04

def select_channel(mask):
    bus.write_byte(TCA_ADDR, mask)

# CH3 ��I��
select_channel(CH2)

# I2C ������
i2c = busio.I2C(board.SCL, board.SDA)

# VL53L1X ������
sensor = adafruit_vl53l1x.VL53L1X(i2c)

sensor.start_ranging()

while True:
    print(sensor.distance, "cm")
    time.sleep(0.1)
