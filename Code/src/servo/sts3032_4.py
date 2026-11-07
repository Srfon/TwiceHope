"""
sts3032_4.py
Mode2(PWM/Torque) position-control wrapper for STS3032
"""

import serial
import time


class STS3032Bus:
    ADDR_WORK_MODE = 33
    ADDR_TORQUE_ENABLE = 40
    ADDR_GOAL_PWM = 44

    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_PWM = 60
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63

    MODE_TORQUE = 2

    DIR = {
        1: -1,
        2: -1,
        3: 1,
        4: 1,
    }

    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000, timeout=0.03):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

        self.ids = []
        self.home = {}
        self.target_angle = {}
        self.integral = {}
        self.prev_error = {}

        self.kp = 6.0
        self.ki = 0.0
        self.kd = 0.15
        self.max_pwm = 1000

        self.scan_ids()

    def checksum(self, data):
        return (~sum(data)) & 0xFF

    def send(self, packet):
        self.ser.write(packet)

    def receive(self, size=64):
        return self.ser.read(size)

    def ping(self, sid):
        pkt = [0xFF, 0xFF, sid, 0x02, 0x01]
        pkt.append(self.checksum(pkt[2:]))
        self.ser.reset_input_buffer()
        self.send(bytes(pkt))
        return len(self.receive()) > 0

    def scan_ids(self):
        self.ids.clear()
        for sid in range(1, 254):
            try:
                if self.ping(sid):
                    self.ids.append(sid)
                    print("FOUND", sid)
                if sid > 5:
                    break
            except Exception:
                pass
        print("AVAILABLE IDS:", self.ids)

    def write_u8(self, sid, addr, value):
        pkt = [0xFF, 0xFF, sid, 0x04, 0x03, addr, value & 0xFF]
        pkt.append(self.checksum(pkt[2:]))
        self.send(bytes(pkt))
        self.receive()

    def write_i16(self, sid, addr, value):
        value = int(value)
        pkt = [
            0xFF, 0xFF, sid, 0x05, 0x03, addr,
            value & 0xFF,
            (value >> 8) & 0xFF
        ]
        pkt.append(self.checksum(pkt[2:]))
        self.send(bytes(pkt))
        self.receive()

    def read(self, sid, addr, size):
        pkt = [0xFF, 0xFF, sid, 0x04, 0x02, addr, size]
        pkt.append(self.checksum(pkt[2:]))
        self.ser.reset_input_buffer()
        self.send(bytes(pkt))
        resp = self.receive()
        if len(resp) < 7:
            return None
        return resp[5:-1]

    def read_u8(self, sid, addr):
        d = self.read(sid, addr, 1)
        return None if d is None else d[0]

    def read_u16(self, sid, addr):
        d = self.read(sid, addr, 2)
        if d is None:
            return None
        return d[0] | (d[1] << 8)

    def sync_write(self, start_addr, data_len, servo_data):
        params = [start_addr, data_len]
        for sid, data in servo_data:
            params.append(sid)
            params.extend(data)

        length = len(params) + 2
        pkt = [0xFF, 0xFF, 0xFE, length, 0x83]
        pkt.extend(params)
        pkt.append(self.checksum(pkt[2:]))
        self.send(bytes(pkt))

    def enable(self):
        for sid in self.ids:
            self.write_u8(sid, self.ADDR_WORK_MODE, self.MODE_TORQUE)
            self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 1)

    def zero(self, sid):
        pos = self.get_position(sid)
        self.home[sid] = pos
        self.target_angle[sid] = 0.0
        self.integral[sid] = 0.0
        self.prev_error[sid] = 0.0

    def zero_all(self):
        for sid in self.ids:
            self.zero(sid)

    def get_position(self, sid):
        return self.read_u16(sid, self.ADDR_PRESENT_POSITION)

    def get_voltage(self, sid):
        v = self.read_u8(sid, self.ADDR_PRESENT_VOLTAGE)
        return None if v is None else v / 10.0

    def get_temperature(self, sid):
        return self.read_u8(sid, self.ADDR_PRESENT_TEMPERATURE)

    def get_pwm(self, sid):
        raw = self.read_u16(sid, self.ADDR_PRESENT_PWM)
        if raw is None:
            return None
        return raw - 65536 if raw > 32767 else raw

    def get_angle(self, sid):
        pos = self.get_position(sid)
        if pos is None:
            return None
        home = self.home.get(sid, pos)
        return ((pos - home) * 360.0 / 4096.0) * self.DIR.get(sid, 1)

    def move(self, targets):
        for sid, deg in targets.items():
            self.target_angle[sid] = float(deg)

    def move_relative(self, targets):
        for sid, deg in targets.items():
            self.target_angle[sid] = self.target_angle.get(sid, 0.0) + float(deg)

    def on(self, pwm_dict):
        data = []
        for sid, pwm in pwm_dict.items():
            pwm = max(-1000, min(1000, int(pwm)))
            data.append((sid, [pwm & 0xFF, (pwm >> 8) & 0xFF]))
        self.sync_write(self.ADDR_GOAL_PWM, 2, data)

    def off(self):
        self.on({sid: 0 for sid in self.ids})

    def update(self):
        pwm_data = []

        for sid in self.ids:
            current = self.get_angle(sid)
            if current is None:
                continue

            target = self.target_angle.get(sid, 0.0)

            err = target - current
            self.integral[sid] = self.integral.get(sid, 0.0) + err
            diff = err - self.prev_error.get(sid, 0.0)

            pwm = (
                self.kp * err +
                self.ki * self.integral[sid] +
                self.kd * diff
            )

            self.prev_error[sid] = err

            if abs(err) < 1.0:
                pwm = 0

            pwm = int(max(-self.max_pwm, min(self.max_pwm, pwm)))
            pwm_data.append((sid, [pwm & 0xFF, (pwm >> 8) & 0xFF]))

        if pwm_data:
            self.sync_write(self.ADDR_GOAL_PWM, 2, pwm_data)

    def close(self):
        self.ser.close()


if __name__ == "__main__":
    motor = STS3032Bus("/dev/ttyUSB0")
    motor.enable()
    motor.zero_all()

    motor.move({
        1: 90,
        2: -90,
        3: 45,
        4: 0,
    })

    try:
        while True:
            motor.update()
            time.sleep(0.02)
    except KeyboardInterrupt:
        motor.off()
        motor.close()
