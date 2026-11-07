#!/usr/bin/env python3

import serial
import time

DEBUG = False

class STS3215:

    # ============================================================
    # Protocol
    # ============================================================

    HEADER = bytes([0xFF, 0xFF])

    INST_PING  = 0x01
    INST_READ  = 0x02
    INST_WRITE = 0x03

    BROADCAST_ID = 0xFE

    # ============================================================
    # STS3215 RAM addresses
    # ============================================================

    ADDR_TORQUE_ENABLE    = 40
    ADDR_ACCELERATION     = 41
    ADDR_GOAL_POSITION    = 42
    ADDR_GOAL_TIME        = 44
    ADDR_GOAL_SPEED        = 46
    ADDR_TORQUE_LIMIT      = 48

    ADDR_LOCK              = 55

    ADDR_PRESENT_POSITION  = 56
    ADDR_PRESENT_SPEED     = 58
    ADDR_PRESENT_LOAD      = 60
    ADDR_PRESENT_VOLTAGE   = 62
    ADDR_PRESENT_TEMP      = 63
    ADDR_MOVING            = 66
    ADDR_PRESENT_CURRENT   = 69

    # ============================================================
    # Initialization
    # ============================================================

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1_000_000,
        timeout=0.05
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    # ============================================================
    # Serial
    # ============================================================

    def open(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout
        )

        # 念のためRXバッファをクリア
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    # ============================================================
    # Checksum
    # ============================================================

    @staticmethod
    def checksum(data):
        """
        ID ～ PARAMETER までの8bit総和を
        bitwise NOTした値。
        """

        return (~sum(data)) & 0xFF

    # ============================================================
    # Packet
    # ============================================================

    def build_packet(self, servo_id, instruction, params=b""):

        length = len(params) + 2

        body = bytes([
            servo_id,
            length,
            instruction
        ]) + params

        checksum = self.checksum(body)

        return self.HEADER + body + bytes([checksum])

    # ============================================================
    # Send
    # ============================================================

    def send_packet(self, packet):

        if self.ser is None:
            raise RuntimeError("Serial port is not open")

        self.ser.reset_input_buffer()

        self.ser.write(packet)
        self.ser.flush()

    # ============================================================
    # Receive status packet
    # ============================================================

    def receive_packet(self):

        if self.ser is None:
            raise RuntimeError("Serial port is not open")

        # --------------------------------------------------------
        # Header FF FF
        # --------------------------------------------------------

        header = self.ser.read(2)

        if len(header) != 2:
            return None

        if header != self.HEADER:
            return None

        # --------------------------------------------------------
        # ID + LENGTH
        # --------------------------------------------------------

        head = self.ser.read(2)

        if len(head) != 2:
            return None

        servo_id = head[0]
        length = head[1]

        # lengthにはERROR + PARAMETER + CHECKSUMが含まれる
        rest = self.ser.read(length)

        if len(rest) != length:
            return None

        packet = self.HEADER + head + rest

        # --------------------------------------------------------
        # Checksum validation
        # --------------------------------------------------------

        checksum_received = packet[-1]
        checksum_calc = self.checksum(packet[2:-1])

        if checksum_received != checksum_calc:
            raise RuntimeError(
                f"Checksum error: "
                f"received=0x{checksum_received:02X}, "
                f"calculated=0x{checksum_calc:02X}"
            )

        return packet

    # ============================================================
    # PING
    # ============================================================

    def ping(self, servo_id):

        packet = self.build_packet(
            servo_id,
            self.INST_PING
        )

        self.send_packet(packet)

        response = self.receive_packet()

        if response is None:
            return False

        return response[2] == servo_id

    # ============================================================
    # READ
    # ============================================================

    def read(self, servo_id, address, length):

        params = bytes([
            address,
            length
        ])

        packet = self.build_packet(
            servo_id,
            self.INST_READ,
            params
        )
        if DEBUG:
            print(
                f"[READ TX] "
                f"ID={servo_id} "
                f"ADDR={address} "
                f"LEN={length} "
                f"| {' '.join(f'{b:02X}' for b in packet)}"
            )

        self.send_packet(packet)

        response = self.receive_packet()

        if response is None:
            raise TimeoutError(
                f"No response from servo ID={servo_id}"
            )
        
        if DEBUG:
            print(
                f"[READ RX] "
                f"{' '.join(f'{b:02X}' for b in response)}"
            )

        # ------------------------------------------------------------
        # Minimum status packet
        #
        # FF FF ID LENGTH ERROR CHECKSUM
        # ------------------------------------------------------------

        if len(response) < 6:
            raise RuntimeError(
                f"Invalid status packet: "
                f"{response.hex(' ')}"
            )

        # ------------------------------------------------------------
        # Header
        # ------------------------------------------------------------

        if response[0] != 0xFF or response[1] != 0xFF:
            raise RuntimeError(
                f"Invalid header: "
                f"{response.hex(' ')}"
            )

        # ------------------------------------------------------------
        # ID
        # ------------------------------------------------------------

        if response[2] != servo_id:
            raise RuntimeError(
                f"Unexpected servo ID: "
                f"expected={servo_id}, "
                f"received={response[2]}"
            )

        # ------------------------------------------------------------
        # LENGTH
        # ------------------------------------------------------------

        packet_length = response[3]

        expected_total_length = 4 + packet_length

        if len(response) != expected_total_length:

            raise RuntimeError(
                f"Invalid packet length: "
                f"declared={packet_length}, "
                f"actual_total={len(response)}, "
                f"packet={response.hex(' ')}"
            )

        # ------------------------------------------------------------
        # ERROR
        # ------------------------------------------------------------

        error = response[4]

        if error != 0:

            raise RuntimeError(
                f"Servo ID={servo_id} returned error "
                f"0x{error:02X}"
            )

        # ------------------------------------------------------------
        # PARAMETER
        # ------------------------------------------------------------

        data = response[5:-1]

        if len(data) != length:

            raise RuntimeError(
                f"Invalid READ data length: "
                f"requested={length}, "
                f"received={len(data)}, "
                f"data={data.hex(' ')}"
            )

        return data

    # ============================================================
    # WRITE
    # ============================================================

    def write(self, servo_id, address, data):

        if isinstance(data, int):
            data = bytes([data])

        data = bytes(data)

        params = bytes([address]) + data

        packet = self.build_packet(
            servo_id,
            self.INST_WRITE,
            params
        )
        if DEBUG:
            print(
                f"[WRITE TX] "
                f"ID={servo_id} "
                f"ADDR={address} "
                f"DATA={data.hex(' ')} "
                f"| {' '.join(f'{b:02X}' for b in packet)}"
            )

        self.send_packet(packet)

        # ------------------------------------------------------------
        # WRITE Status Packetを待つ
        # ------------------------------------------------------------

        response = self.receive_packet()

        if response is None:
            raise TimeoutError(
                f"No WRITE response from servo ID={servo_id}"
            )

        if DEBUG:
            print(
                f"[WRITE RX] "
                f"{' '.join(f'{b:02X}' for b in response)}"
            )

        # ------------------------------------------------------------
        # Validate
        # ------------------------------------------------------------

        if len(response) < 6:
            raise RuntimeError(
                f"Invalid WRITE response: "
                f"{response.hex(' ')}"
            )

        if response[0] != 0xFF or response[1] != 0xFF:
            raise RuntimeError(
                f"Invalid WRITE response header: "
                f"{response.hex(' ')}"
            )

        if response[2] != servo_id:
            raise RuntimeError(
                f"Unexpected WRITE response ID: "
                f"expected={servo_id}, "
                f"received={response[2]}"
            )

        # ------------------------------------------------------------
        # LENGTH
        # ------------------------------------------------------------

        packet_length = response[3]

        expected_total_length = 4 + packet_length

        if len(response) != expected_total_length:
            raise RuntimeError(
                f"Invalid WRITE response length: "
                f"declared={packet_length}, "
                f"actual={len(response)}, "
                f"packet={response.hex(' ')}"
            )

        # ------------------------------------------------------------
        # ERROR
        # ------------------------------------------------------------

        error = response[4]

        if error != 0:
            raise RuntimeError(
                f"Servo ID={servo_id} WRITE error "
                f"0x{error:02X}"
            )

    # ============================================================
    # 1 byte
    # ============================================================

    def read_u8(self, servo_id, address):

        data = self.read(
            servo_id,
            address,
            1
        )

        return data[0]

    def write_u8(self, servo_id, address, value):

        if not 0 <= value <= 255:
            raise ValueError("u8 value out of range")

        self.write(
            servo_id,
            address,
            bytes([value])
        )

    # ============================================================
    # 2 byte
    # ============================================================

    def read_u16(self, servo_id, address):

        data = self.read(
            servo_id,
            address,
            2
        )

        if len(data) != 2:
            raise RuntimeError(
                f"Invalid u16 response: "
                f"ID={servo_id}, "
                f"address={address}, "
                f"data={data.hex(' ')}, "
                f"length={len(data)}"
            )

        return data[0] | (data[1] << 8)

    def write_u16(self, servo_id, address, value):

        if not 0 <= value <= 0xFFFF:
            raise ValueError("u16 value out of range")

        data = bytes([
            value & 0xFF,
            (value >> 8) & 0xFF
        ])

        self.write(
            servo_id,
            address,
            data
        )

    # ============================================================
    # Servo scan
    # ============================================================

    def scan(self, start_id=0, end_id=20):

        found = []

        for servo_id in range(start_id, end_id + 1):

            try:

                if self.ping(servo_id):
                    found.append(servo_id)

            except Exception:
                pass

        return found

    # ============================================================
    # Mode
    # ============================================================

    ADDR_MODE = 33

    MODE_POSITION = 0
    MODE_VELOCITY = 1
    MODE_PWM      = 2
    MODE_STEP     = 3

    def get_mode(self, servo_id):
        return self.read_u8(
            servo_id,
            self.ADDR_MODE
        )

    def set_mode(self, servo_id, mode):

        if mode not in (
            self.MODE_POSITION,
            self.MODE_VELOCITY,
            self.MODE_PWM,
            self.MODE_STEP
        ):
            raise ValueError(
                f"Invalid mode: {mode}"
            )

        # ModeはEEPROM側設定なので、
        # torque OFFで変更する
        self.torque(servo_id, False)

        self.write_u8(
            servo_id,
            self.ADDR_MODE,
            mode
        )


    # ============================================================
    # Acceleration
    # ============================================================

    def set_acceleration(self, servo_id, acceleration):

        if not 0 <= acceleration <= 254:
            raise ValueError(
                "Acceleration must be 0..254"
            )

        self.write_u8(
            servo_id,
            self.ADDR_ACCELERATION,
            acceleration
        )


    # ============================================================
    # Goal Time
    # ============================================================

    def set_goal_time(self, servo_id, time_ms):

        if not 0 <= time_ms <= 65535:
            raise ValueError(
                "Goal time must be 0..65535 ms"
            )

        self.write_u16(
            servo_id,
            self.ADDR_GOAL_TIME,
            time_ms
        )


    # ============================================================
    # Hardware error
    # ============================================================

    def get_hardware_error(self, servo_id):

        return self.read_u8(
            servo_id,
            65
        )


    # ============================================================
    # Load
    # ============================================================

    def get_load(self, servo_id):

        return self.read_u16(
            servo_id,
            self.ADDR_PRESENT_LOAD
        )


    # ============================================================
    # Current
    # ============================================================

    def get_current(self, servo_id):

        return self.read_u16(
            servo_id,
            self.ADDR_PRESENT_CURRENT
        )


    # ============================================================
    # Servo status
    # ============================================================

    def status(self, servo_id):

        return {
            "id": servo_id,
            "position": self.get_position(servo_id),
            "speed": self.get_speed(servo_id),
            "load": self.get_load(servo_id),
            "voltage": self.get_voltage(servo_id),
            "temperature": self.get_temperature(servo_id),
            "moving": self.is_moving(servo_id),
            "current": self.get_current(servo_id),
            "hardware_error": self.get_hardware_error(servo_id),
            "mode": self.get_mode(servo_id),
        }

    # ============================================================
    # Torque
    # ============================================================

    def torque(self, servo_id, enable):

        self.write_u8(
            servo_id,
            self.ADDR_TORQUE_ENABLE,
            1 if enable else 0
        )

    # ============================================================
    # Goal Speed
    # ============================================================

    def set_speed(self, servo_id, speed):

        self.write_u16(
            servo_id,
            self.ADDR_GOAL_SPEED,
            speed
        )

    # ============================================================
    # Goal Position
    # ============================================================

    def set_position(self, servo_id, position):

        if not 0 <= position <= 4095:
            raise ValueError(
                "Position must be 0..4095"
            )

        self.write_u16(
            servo_id,
            self.ADDR_GOAL_POSITION,
            position
        )

    # ============================================================
    # Present Position
    # ============================================================

    def get_position(self, servo_id):

        return self.read_u16(
            servo_id,
            self.ADDR_PRESENT_POSITION
        )

    # ============================================================
    # Present Speed
    # ============================================================

    def get_speed(self, servo_id):

        return self.read_u16(
            servo_id,
            self.ADDR_PRESENT_SPEED
        )

    # ============================================================
    # Voltage
    # ============================================================

    def get_voltage(self, servo_id):

        raw = self.read_u8(
            servo_id,
            self.ADDR_PRESENT_VOLTAGE
        )

        return raw * 0.1

    # ============================================================
    # Temperature
    # ============================================================

    def get_temperature(self, servo_id):

        return self.read_u8(
            servo_id,
            self.ADDR_PRESENT_TEMP
        )

    # ============================================================
    # Moving
    # ============================================================

    def is_moving(self, servo_id):

        return bool(
            self.read_u8(
                servo_id,
                self.ADDR_MOVING
            )
        )


if __name__ == "__main__":

    servo = STS3215("/dev/ttyUSB0")

    try:

        servo.open()

        print("STS3215 driver opened")

        print("Scanning...")

        ids = servo.scan(0, 20)

        print("Detected IDs:", ids)

    finally:

        servo.close()