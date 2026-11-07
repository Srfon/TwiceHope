import serial
import time


class STSBus:
    HEADER = [0xFF, 0xFF]

    # --- インストラクション ---
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_SYNC_WRITE = 0x83

    # --- EEPROM 領域 ---
    ADDR_MODE = 33          # 動作モード(WorkMode)
    ADDR_LOCK = 55          # EEPROM ロック (0=解除, 1=ロック)

    # --- SRAM 領域 ---
    ADDR_TORQUE_ENABLE = 40
    ADDR_GOAL_ACC = 41      # mode=2 のトルク/PWM 指定(記事の実測アドレス)
    ADDR_GOAL_POSITION = 42  # mode=0:目標位置 / mode=3:移動量
    ADDR_MOVE_TIME = 44
    ADDR_MOVE_SPEED = 46    # mode=1:速度 / mode=3:速度

    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63

    # --- 動作モード定数 ---
    MODE_SERVO = 0
    MODE_SPEED = 1
    MODE_TORQUE = 2
    MODE_STEP = 3

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1000000,
        timeout=0.03,
        mode=MODE_SERVO,
        ids=None,
        auto_scan=True,
    ):
        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=timeout,
        )

        self.ids = []
        self.mode = mode

        # ステップモードで送った累計移動量(多回転の現在位置の目安)。
        self._step_total = {}

        print(f"STSBus opened : {port}")

        if ids is not None:
            self.ids = list(ids)
            print(f"Target IDs (manual): {self.ids}")
        elif auto_scan:
            self.scan_ids()

    # =====================================================
    # 基本
    # =====================================================

    def checksum(self, data):
        return (~sum(data)) & 0xFF

    def send(self, packet):
        print("TX:", packet.hex(" "))
        self.ser.write(packet)

    def receive(self, size=64):
        data = self.ser.read(size)
        if data:
            print("RX:", data.hex(" "))
        return data

    def close(self):
        self.ser.close()

    @staticmethod
    def _enc_signed(value):
        """STS の符号付き 16bit(符号は bit15、下位15bitが大きさ)に変換。"""
        value = int(value)
        if value < 0:
            return (abs(value) & 0x7FFF) | 0x8000
        return value & 0x7FFF

    # =====================================================
    # PING / SCAN
    # =====================================================

    def ping(self, sid):
        packet = [0xFF, 0xFF, sid, 0x02, self.INST_PING]
        packet.append(self.checksum(packet[2:]))

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
                    print(f"FOUND ID {sid}")
                if sid > 5:
                    break
            except Exception:
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
        packet = [0xFF, 0xFF, sid, 0x04, self.INST_READ, address, size]
        packet.append(self.checksum(packet[2:]))

        self.ser.reset_input_buffer()
        self.send(bytes(packet))
        resp = self.receive()

        if len(resp) < 7:
            return None

        return resp[5:-1]

    def read_u8(self, sid, address):
        data = self.read(sid, address, 1)
        if not data:
            return None
        return data[0]

    def read_u16(self, sid, address):
        data = self.read(sid, address, 2)
        if data is None or len(data) < 2:
            return None
        return data[0] | (data[1] << 8)

    def get_position(self, sid):
        return self.read_u16(sid, self.ADDR_PRESENT_POSITION)

    def get_mode(self, sid):
        return self.read_u8(sid, self.ADDR_MODE)


    def sync_write(self, start_addr, data_len, servo_data):
        params = [start_addr, data_len]
        for sid, data in servo_data:
            params.append(sid)
            params.extend(data)

        length = len(params) + 2

        packet = [0xFF, 0xFF, 0xFE, length, self.INST_SYNC_WRITE]
        packet.extend(params)
        packet.append(self.checksum(packet[2:]))

        self.send(bytes(packet))

    # =====================================================
    # WRITE
    # =====================================================

    def write_u8(self, sid, address, value):
        packet = [
            0xFF, 0xFF, sid, 0x04, self.INST_WRITE,
            address, value & 0xFF,
        ]
        packet.append(self.checksum(packet[2:]))
        self.send(bytes(packet))
        self.receive()

    def write_u16(self, sid, address, value):
        packet = [
            0xFF, 0xFF, sid, 0x05, self.INST_WRITE,
            address, value & 0xFF, (value >> 8) & 0xFF,
        ]
        packet.append(self.checksum(packet[2:]))
        self.send(bytes(packet))
        self.receive()

    # =====================================================
    # 動作モード切替 (EEPROM addr 33)
    # =====================================================

    def set_mode(self, sid, mode, force=False):
        """
        サーボの動作モードを切り替える。
        addr 33 は EEPROM なので、ロック解除 -> 書込み -> 再ロック を行う。
        現在値と同じなら(EEPROM 寿命保護のため)書き込まない。
        """
        if not force:
            current = self.get_mode(sid)
            if current is not None and current == mode:
                self.mode = mode
                return

        self.torque_off(sid)                       # モード変更前はトルクOFF
        self.write_u8(sid, self.ADDR_LOCK, 0)      # EEPROM ロック解除
        self.write_u8(sid, self.ADDR_MODE, mode)   # 動作モード書込み
        self.write_u8(sid, self.ADDR_LOCK, 1)      # EEPROM 再ロック
        self.torque_on(sid)                        # 再度トルクON

        self.mode = mode

        print(f"ID={sid} MODE -> {mode}")

    def set_mode_all(self, mode, force=False):
        for sid in self.ids:
            self.set_mode(sid, mode, force=force)

    # =====================================================
    # トルク
    # =====================================================

    def torque_on(self, sid):
        self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 1)

    def torque_off(self, sid):
        self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 0)

    # =====================================================
    # トルクモード専用制御  (mode=2, Address 33 = 2 を前提)
    # =====================================================

    def torque_free(self, servo_torques) -> None:
        """
        servo_torques:
            [
                (1, 300),
                (2, -300),
                (3, 0),
            ]
        """
        items = list(servo_torques)
        servo_data = []
        for sid, torque in items:
            enc = self._enc_signed(torque)
            servo_data.append(
                (
                    sid,
                    [
                        enc & 0xFF,
                        (enc >> 8) & 0xFF,
                    ],
                )
            )
        self.sync_write(
            self.ADDR_MOVE_TIME,
            2,
            servo_data,
        )
        print(f"torque_free {items}")

    def torque_rotate_degrees(
        self,
        servo_torques,
        degrees: float,
        poll_interval: float = 0.01,
    ) -> None:
        items = list(servo_torques)
        ids = [
            sid
            for sid, _
            in items
        ]
        torque_map = {
            sid: torque
            for sid, torque
            in items
        }
        prev = {}
        accumulated = {}
        for sid in ids:
            pos = self.get_position(sid)
            if pos is None:
                raise RuntimeError(
                    f"ID={sid} position read failed"
                )
            prev[sid] = pos
            accumulated[sid] = 0
        self.torque_free(items)
        active = set(ids)
        while active:
            time.sleep(poll_interval)
            stop_ids = []
            for sid in list(active):
                pos = self.get_position(sid)
                if pos is None:
                    continue
                diff = pos - prev[sid]
                if diff < -2048:
                    diff += 4096
                elif diff > 2048:
                    diff -= 4096
                direction = (
                    1
                    if torque_map[sid] >= 0
                    else -1
                )
                accumulated[sid] += (
                    diff * direction
                )
                prev[sid] = pos
                moved_deg = (
                    accumulated[sid]
                    * 360.0
                    / 4096.0
                )
                print(
                    f"ID={sid} "
                    f"{moved_deg:.1f}°/"
                    f"{degrees}°"
                )
                if moved_deg >= degrees:
                    stop_ids.append(sid)
            if stop_ids:
                stop_cmd = [
                    (sid, 0)
                    for sid in stop_ids
                ]
                self.torque_free(stop_cmd)
                for sid in stop_ids:
                    active.remove(sid)
                    print(
                        f"ID={sid} stop"
                    )
        print(
            "torque_rotate_degrees complete"
        )

    def torque_rotate_time(
        self,
        servo_torques,
        time_ms: int,
    ) -> None:
        """
        例
        bus.torque_rotate_time(
            [
                (1, 300),
                (2, -300),
            ],
            2000
        )
        """
        items = list(servo_torques)
        self.torque_free(items)
        print(
            f"torque_rotate_time "
            f"{items} "
            f"{time_ms}ms"
        )
        time.sleep(time_ms / 1000)
        stop_cmd = [
            (sid, 0)
            for sid, _ in items
        ]
        self.torque_free(stop_cmd)
        print("torque_rotate_time complete")

def main():
    bus = STSBus()
    bus.torque_off(1)
    bus.torque_off(2)
    bus.set_mode_all(2)
    bus.torque_rotate_time([(1, 300), (2, 300)], 1000)
    bus.torque_on(1)
    bus.torque_on(2)
    bus.close()

if __name__ == "__main__":
    main()