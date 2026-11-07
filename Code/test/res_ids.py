import serial, time

ser = serial.Serial('/dev/ttyAMA0', 1000000, timeout=0.1)

def ping(id):
    pkt = bytes([0xFF,0xFF,id,0x02,0x01, (~(id+0x02+0x01)&0xFF)])
    ser.reset_input_buffer()
    ser.write(pkt)
    time.sleep(0.01)
    return ser.read(6)

for i in range(1,10):
    print(i, ping(i))
