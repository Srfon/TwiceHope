import serial

ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)

while True:
    line = ser.readline().decode(errors="ignore").strip()

    if not line:
        continue

    if not line.startswith("IMU:"):
        continue 

    print("RAW:", line)

    try:
        parts = line.split(";")

        imu = parts[0].replace("IMU:", "").split(",")
        c1  = parts[1].replace("C1:", "").split(",")
        c2  = parts[2].replace("C2:", "").split(",")

        yaw, pitch, roll = map(float, imu)
        r1, g1, b1 = map(int, c1)
        r2, g2, b2 = map(int, c2)

        print("IMU:", yaw, pitch, roll)
        print("C1:", r1, g1, b1)
        print("C2:", r2, g2, b2)

    except Exception as e:
        print("Parse error:", e)
