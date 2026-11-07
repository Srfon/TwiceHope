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
            # ▼ 自分でIDを指定したとき: スキャンせずこのIDだけを使う
            self.ids = list(ids)
            print(f"Target IDs (manual): {self.ids}")
            self.set_mode_all(mode)
        elif auto_scan:
            # ▼ 指定なし: 自動探索して見つかった全サーボを使う
            self.scan_ids()
            self.set_mode_all(mode)

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

    def torque_free(self, ids: list[int], torque: int) -> None:
        """
        指定 ID をトルク値でフリー回転させる。
        Address 44(Goal Time / PWM) に torque を書き込むだけ。
        停止は torque=0 で再呼び出し。

            bus.torque_free([1, 2], 300)   # 正回転
            bus.torque_free([1, 2], -300)  # 逆回転
            bus.torque_free([1, 2], 0)     # 停止

        ※ mode=2(トルクモード) が Address 33 に設定済みであること。
        """
        enc = self._enc_signed(torque)
        lo, hi = enc & 0xFF, (enc >> 8) & 0xFF
        servo_data = [(sid, [lo, hi]) for sid in ids]
        self.sync_write(self.ADDR_MOVE_TIME, 2, servo_data)  # addr 44
        print(f"torque_free  ids={ids}  torque={torque}")

    def torque_rotate_degrees(
        self, ids: list[int], torque: int, degrees: float,
        poll_interval: float = 0.01,
    ) -> None:
        """
        指定 ID をトルクで回転させ、各 ID がスタート時から
        degrees 以上回ったら個別に停止する。全 ID 停止でリターン。

            bus.torque_rotate_degrees([1, 2], 300, 180)   # 各180度回転
            bus.torque_rotate_degrees([1, 2], -300, 90)   # 逆回転 90度

        移動量の計算:
            - get_position() の生値 0〜4095 を毎ループ取得する。
            - 前回値との差が -2048 未満(大きく減少)なら 0→4095 の
              折り返しが発生したと判断して +4096 補正する。
            - 逆に差が +2048 超なら 4095→0 の折り返しで -4096 補正する。
            - この補正を積算した raw_steps を角度に換算する:
                angle = raw_steps * (360 / 4096)
            - torque の符号で回転方向を決め、その方向の積算値が
              degrees に達したら停止と判定する。

        poll_interval : ポーリング間隔(秒)。短いほど精度が上がるが
                        シリアル通信の負荷が増える。既定 0.01s(10ms)。
        """
        ids = list(ids)
        direction = 1 if torque >= 0 else -1   # +1:正転 / -1:逆転

        # スタート位置を記録
        prev: dict[int, int]      = {}
        accumulated: dict[int, float] = {}
        for sid in ids:
            pos = self.get_position(sid)
            if pos is None:
                raise RuntimeError(f"ID={sid} の位置取得に失敗しました")
            prev[sid]        = pos
            accumulated[sid] = 0.0

        # 全 ID 同時スタート
        self.torque_free(ids, torque)
        print(f"torque_rotate_degrees  ids={ids}  torque={torque}  target={degrees}°")

        active = set(ids)   # まだ動いている ID の集合

        while active:
            time.sleep(poll_interval)

            stopped_now: list[int] = []
            for sid in list(active):
                pos = self.get_position(sid)
                if pos is None:
                    continue

                # 0〜4095 の折り返しを補正した差分を積算
                diff = pos - prev[sid]
                if diff < -2048:
                    diff += 4096   # 4095→0 の正転折り返し
                elif diff > 2048:
                    diff -= 4096   # 0→4095 の逆転折り返し

                accumulated[sid] += diff * direction   # 方向に合わせた積算
                prev[sid] = pos

                moved_deg = accumulated[sid] * (360 / 4096)
                print(
                    f"  ID={sid}  raw={pos}  moved={moved_deg:.1f}°"
                    f"  target={degrees}°"
                )

                if moved_deg >= degrees:
                    stopped_now.append(sid)

            # 目標達成 ID を停止
            for sid in stopped_now:
                self.torque_free([sid], 0)
                active.discard(sid)
                print(f"  ID={sid} STOP  (moved={accumulated[sid] * (360/4096):.1f}°)")

        print("torque_rotate_degrees 完了: 全 ID 停止")

    def torque_rotate_time(
        self, ids: list[int], torque: int, time_ms: int
    ) -> None:
        """
        指定 ID をトルクで回転させ、time_ms ミリ秒後に停止する。

            bus.torque_rotate_time([1, 2], 300, 2000)   # 2秒回転
            bus.torque_rotate_time([1, 2], -300, 500)   # 逆回転 0.5秒

        ※ time.sleep の精度(OS スケジューラ依存)に依存する。
           高精度が必要なら torque_rotate_degrees を使うこと。
        """
        self.torque_free(ids, torque)
        print(f"torque_rotate_time  ids={ids}  torque={torque}  wait={time_ms}ms")

        time.sleep(time_ms / 1000)

        self.torque_free(ids, 0)
        print(f"torque_rotate_time 完了: {time_ms}ms 経過 → 全 ID 停止")

def main():
    bus = STSBus()
    bus.torque_rotate_time([1, 2], 300, 2000)
    bus.close()

if __name__ == "__main__":
    main()
