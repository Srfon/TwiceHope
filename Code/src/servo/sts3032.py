import serial
import time


class STS3032:

    HEADER = [0xFF, 0xFF]

    # 命令
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_SYNC_WRITE = 0x83

    # アドレス
    ADDR_GOAL_POSITION = 42
    ADDR_PRESENT_POSITION = 56

    # RAM

    ADDR_ACC = 41

    ADDR_TORQUE_ENABLE = 40

    ADDR_GOAL_POSITION = 42

    ADDR_PRESENT_POSITION = 56

    ADDR_PRESENT_VOLTAGE = 62

    ADDR_PRESENT_TEMPERATURE = 63

    ADDR_MOVE_TIME = 44

    ADDR_MOVE_SPEED = 46

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1000000,
        servo_id=1,
        timeout=0.1
    ):

        self.id = servo_id

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )

        print("Servo opened, port:", port)

    def write_bytes(
        self,
        servo_id,
        address,
        data
    ):
        """
        data : list[int]
        """

        length = len(data) + 3

        packet = [

            0xFF,
            0xFF,

            servo_id,

            length,

            self.INST_WRITE,

            address
        ]

        packet.extend(data)

        checksum = self.calc_checksum(
            packet[2:]
        )

        packet.append(checksum)

        packet = bytes(packet)

        print(
            "TX:",
            packet.hex(" ")
        )

        self.ser.write(packet)

        return self.receive_packet()

    # ==================================================
    # チェックサム
    # ==================================================

    def calc_checksum(self, data):

        return (~sum(data)) & 0xFF

    # ==================================================
    # パケット生成
    # ==================================================

    def build_sync_move_packet(
        self,
        motions
    ):
        params = []

        for sid, cfg in motions.items():

            pos = cfg["pos"]

            speed = cfg.get(
                "speed",
                0
            )

            acc = cfg.get(
                "acc",
                0
            )

            time_ms = cfg.get(
                "time",
                0
            )

            params.extend([

                sid,

                acc,

                pos & 0xFF,
                (pos >> 8) & 0xFF,

                time_ms & 0xFF,
                (time_ms >> 8) & 0xFF,

                speed & 0xFF,
                (speed >> 8) & 0xFF
            ])
        packet = [

            0xFF,
            0xFF,

            0xFE,

            len(params) + 4,

            self.INST_SYNC_WRITE,

            self.ADDR_ACC,

            7
        ]

        packet.extend(params)

        checksum = self.calc_checksum(
            packet[2:]
        )

        packet.append(checksum)

        return bytes(packet)

    def build_read_packet(
        self,
        address,
        size
    ):

        packet = [
            0xFF,
            0xFF,
            self.id,
            0x04,
            self.INST_READ,
            address,
            size
        ]

        checksum = self.calc_checksum(packet[2:])
        packet.append(checksum)

        return bytes(packet)

    def build_write_packet(
        self,
        address,
        value,
        size=2
    ):

        params = [address]

        if size == 1:

            params.append(value & 0xFF)

        elif size == 2:

            params.extend([
                value & 0xFF,
                (value >> 8) & 0xFF
            ])

        length = len(params) + 2

        packet = [
            0xFF,
            0xFF,
            self.id,
            length,
            self.INST_WRITE
        ]

        packet.extend(params)

        checksum = self.calc_checksum(packet[2:])
        packet.append(checksum)

        return bytes(packet)

    # ==================================================
    # 通信
    # ==================================================

    def send_packet(self, packet):

        self.ser.reset_input_buffer()

        print("TX:", packet.hex(" "))

        self.ser.write(packet)

    def receive_packet(self):

        time.sleep(0.01)

        response = self.ser.read(32)

        print("RX:", response.hex(" "))

        return response

    # ==================================================
    # パケット解析
    # ==================================================

    def parse_status_packet(self, packet):

        if len(packet) < 6:

            raise RuntimeError("packet too short")

        if packet[0] != 0xFF or packet[1] != 0xFF:

            raise RuntimeError("header error")

        servo_id = packet[2]
        length = packet[3]
        error = packet[4]

        params = packet[5:-1]

        return {
            "id": servo_id,
            "length": length,
            "error": error,
            "params": params
        }

    # ==================================================
    # Read
    # ==================================================

    def read_data(
        self,
        address,
        size
    ):

        packet = self.build_read_packet(
            address,
            size
        )

        self.send_packet(packet)

        response = self.receive_packet()

        info = self.parse_status_packet(
            response
        )

        return info["params"]

    def read_u8(
        self,
        address
    ):

        data = self.read_data(
            address,
            1
        )

        return data[0]

    def read_u16(
        self,
        address
    ):

        data = self.read_data(
            address,
            2
        )

        return (
            data[0]
            |
            (data[1] << 8)
        )

    # ==================================================
    # Write
    # ==================================================

    def write_u8(
        self,
        address,
        value
    ):

        packet = self.build_write_packet(
            address,
            value,
            size=1
        )

        self.send_packet(packet)

        return self.receive_packet()

    def write_u16(
        self,
        address,
        value
    ):

        packet = self.build_write_packet(
            address,
            value,
            size=2
        )

        self.send_packet(packet)

        return self.receive_packet()

    # ==================================================
    # STS3032 API
    # ==================================================

    def get_position(self):

        return self.read_u16(
            self.ADDR_PRESENT_POSITION
        )

    def move(
        self,
        servo_id,
        position,
        speed=0,
        acc=0,
        time_ms=0
    ):

        data = [

            acc,

            position & 0xFF,
            (position >> 8) & 0xFF,

            time_ms & 0xFF,
            (time_ms >> 8) & 0xFF,

            speed & 0xFF,
            (speed >> 8) & 0xFF
        ]

        return self.write_bytes(
            servo_id,
            self.ADDR_ACC,
            data
        )

    def sync_move(
        self,
        motions
    ):

        packet = self.build_sync_move_packet(
            motions
        )

        print(
            "TX:",
            packet.hex(" ")
        )

        self.ser.write(packet)

    def move_speed(
        self,
        servo_id,
        position,
        speed,
        acc=0
    ):

        return self.move(
            servo_id,
            position,
            speed=speed,
            acc=acc
        )

    def move_time(
        self,
        servo_id,
        position,
        time_ms,
        acc=0
    ):

        return self.move(
            servo_id,
            position,
            acc=acc,
            time_ms=time_ms
        )

    def wait_until_position(
        self,
        target,
        tolerance=20,
        timeout=5.0
    ):

        start = time.time()

        while True:

            pos = self.get_position()

            if abs(pos - target) <= tolerance:
                return True

            if time.time() - start > timeout:
                return False

            time.sleep(0.01)

    def move_time_block(
        self,
        position,
        time_ms
    ):

        self.move_time(
            position,
            time_ms
        )

        return self.wait_until_position(
            position
        )

    def move_speed_block(
        self,
        position,
        speed
    ):

        self.move_speed(
            position,
            speed
        )

        return self.wait_until_position(
            position
        )

    def get_temperature(self):

        return self.read_u8(
            self.ADDR_PRESENT_TEMPERATURE
        )

    def torque_on(self):

        self.write_u8(
            self.ADDR_TORQUE_ENABLE,
            1
        )

    def torque_off(self):

        self.write_u8(
            self.ADDR_TORQUE_ENABLE,
            0
        )

    def get_voltage(self):

        raw = self.read_u8(
            self.ADDR_PRESENT_VOLTAGE
        )

        return raw / 10.0   

    def ping(self):

        packet = [
            0xFF,
            0xFF,
            self.id,
            0x02,
            self.INST_PING
        ]

        checksum = self.calc_checksum(
            packet[2:]
        )

        packet.append(checksum)

        self.send_packet(
            bytes(packet)
        )

        response = self.receive_packet()

        return len(response) > 0

    @classmethod
    def scan_ids(
        cls,
        port="/dev/ttyUSB0",
        baudrate=1000000
    ):

        found = []

        for sid in range(0, 254):

            try:

                servo = cls(
                    port=port,
                    baudrate=baudrate,
                    servo_id=sid
                )

                if servo.ping():

                    found.append(sid)

                    print(
                        f"FOUND ID {sid}"
                    )

                servo.close()

            except:

                pass

        return found
    def close(self):

        self.ser.close()

    def read_u16_debug(self, addr):

        value = self.read_u16(addr)

        print(
            f"ADDR {addr} = {value}"
        )

        return value