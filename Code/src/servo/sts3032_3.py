import serial
import time


class STSBus:

    HEADER = [0xFF, 0xFF]

    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_SYNC_WRITE = 0x83

    ADDR_TORQUE_ENABLE = 40

    ADDR_GOAL_POSITION = 42
    ADDR_MOVE_TIME = 44
    ADDR_MOVE_SPEED = 46

    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1000000,
        timeout=0.03,
        auto_scan=True
    ):

        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=timeout
        )

        self.ids = []

        print(
            f"STSBus opened : {port}"
        )

        if auto_scan:
            self.scan_ids()

    # =====================================================
    # 基本
    # =====================================================

    def checksum(self, data):

        return (~sum(data)) & 0xFF

    def send(self, packet):

        print(
            "TX:",
            packet.hex(" ")
        )

        self.ser.write(packet)

    def receive(self, size=64):

        data = self.ser.read(size)

        if data:
            print(
                "RX:",
                data.hex(" ")
            )

        return data

    def close(self):

        self.ser.close()

    # =====================================================
    # PING
    # =====================================================

    def ping(self, sid):

        packet = [
            0xFF,
            0xFF,
            sid,
            0x02,
            self.INST_PING
        ]

        packet.append(
            self.checksum(packet[2:])
        )

        self.ser.reset_input_buffer()

        self.send(bytes(packet))

        resp = self.receive()

        return len(resp) > 0

    def scan_ids(self):

        print()
        print("Scanning Servo IDs...")
        print()

        self.ids.clear()

        for sid in range(1, 254):

            try:

                if self.ping(sid):

                    self.ids.append(sid)

                    print(
                        f"FOUND ID {sid}"
                    )
                if sid > 5:
                    break

            except:
                pass

        print()
        print("Available IDs:")
        print(self.ids)
        print()

        return self.ids

    # =====================================================
    # READ
    # =====================================================

    def read(self, sid, address, size):

        packet = [
            0xFF,
            0xFF,
            sid,
            0x04,
            self.INST_READ,
            address,
            size
        ]

        packet.append(
            self.checksum(packet[2:])
        )

        self.ser.reset_input_buffer()

        self.send(bytes(packet))

        resp = self.receive()

        if len(resp) < 7:
            return None

        return resp[5:-1]

    def read_u16(self, sid, address):

        data = self.read(
            sid,
            address,
            2
        )

        if data is None:
            return None

        return data[0] | (data[1] << 8)

    def get_position(self, sid):

        return self.read_u16(
            sid,
            self.ADDR_PRESENT_POSITION
        )

    # =====================================================
    # WRITE
    # =====================================================

    def write_u8(
        self,
        sid,
        address,
        value
    ):

        packet = [
            0xFF,
            0xFF,
            sid,
            0x04,
            self.INST_WRITE,
            address,
            value & 0xFF
        ]

        packet.append(
            self.checksum(packet[2:])
        )

        self.send(bytes(packet))

        self.receive()

    def write_u16(
        self,
        sid,
        address,
        value
    ):

        packet = [
            0xFF,
            0xFF,
            sid,
            0x05,
            self.INST_WRITE,
            address,
            value & 0xFF,
            (value >> 8) & 0xFF
        ]

        packet.append(
            self.checksum(packet[2:])
        )

        self.send(bytes(packet))

        self.receive()

    # =====================================================
    # 個別制御
    # =====================================================

    def move(
        self,
        sid,
        position,
        speed=0,
        time_ms=0
    ):

        position = max(
            0,
            min(4095, position)
        )

        speed = max(
            0,
            min(4095, speed)
        )

        self.write_u16(
            sid,
            self.ADDR_MOVE_TIME,
            time_ms
        )

        self.write_u16(
            sid,
            self.ADDR_MOVE_SPEED,
            speed
        )

        self.write_u16(
            sid,
            self.ADDR_GOAL_POSITION,
            position
        )

        print(
            f"ID={sid} POS={position} SPEED={speed}"
        )

    # =====================================================
    # SYNC WRITE
    # =====================================================

    def sync_write(
        self,
        start_addr,
        data_len,
        servo_data
    ):

        params = [
            start_addr,
            data_len
        ]

        for sid, data in servo_data:

            params.append(sid)
            params.extend(data)

        length = len(params) + 2

        packet = [
            0xFF,
            0xFF,
            0xFE,
            length,
            self.INST_SYNC_WRITE
        ]

        packet.extend(params)

        packet.append(
            self.checksum(packet[2:])
        )

        self.send(bytes(packet))

    # =====================================================
    # 複数同時制御
    # =====================================================

    def move_group(
        self,
        targets
    ):

        """
        targets = {

            1:{
                "position":2048,
                "speed":500,
                "time":0
            },

            2:{
                "position":1000,
                "speed":500,
                "time":0
            }
        }
        """

        servo_data = []

        print()
        print("SYNC MOVE")
        print()

        for sid, cfg in targets.items():

            pos = cfg.get(
                "position",
                2048
            )

            spd = cfg.get(
                "speed",
                0
            )

            movetime = cfg.get(
                "time",
                0
            )

            pos = max(
                0,
                min(4095, pos)
            )

            spd = max(
                0,
                min(4095, spd)
            )

            data = [

                pos & 0xFF,
                (pos >> 8) & 0xFF,

                movetime & 0xFF,
                (movetime >> 8) & 0xFF,

                spd & 0xFF,
                (spd >> 8) & 0xFF
            ]

            servo_data.append(
                (sid, data)
            )

            print(
                f"ID={sid} "
                f"POS={pos} "
                f"SPEED={spd} "
                f"TIME={movetime}"
            )

        self.sync_write(
            start_addr=42,
            data_len=6,
            servo_data=servo_data
        )

    # =====================================================
    # トルク
    # =====================================================

    def torque_on(self, sid):

        self.write_u8(
            sid,
            self.ADDR_TORQUE_ENABLE,
            1
        )

    def torque_off(self, sid):

        self.write_u8(
            sid,
            self.ADDR_TORQUE_ENABLE,
            0
        )


if __name__ == "__main__":

    bus = STSBus(
        "/dev/ttyUSB0"
    )

    bus.move_group({

        1: {
            "position": 1000,
            "speed": 500
        },

        2: {
            "position": 2000,
            "speed": 800
        },

        3: {
            "position": 3000,
            "speed": 1000
        }

    })

    time.sleep(3)

    bus.move(
        sid=1,
        position=2048,
        speed=500
    )

    bus.close()