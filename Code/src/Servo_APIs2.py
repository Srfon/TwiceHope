import serial
import time


class STSBus:
    HEADER = [0xFF, 0xFF]

    # --- インストラクション ---
    INST_PING       = 0x01
    INST_READ       = 0x02
    INST_WRITE      = 0x03
    INST_SYNC_WRITE = 0x83

    # --- EEPROM 領域 ---
    ADDR_MODE = 33   # WorkMode (0=Position 1=Velocity 2=PWM 3=Step)
    ADDR_LOCK = 55   # EEPROM ロック (0=解除, 1=ロック)

    # --- SRAM 領域 ---
    ADDR_TORQUE_ENABLE   = 40   # 0=脱力, 1=トルクON
    ADDR_GOAL_ACC        = 41   # Goal Acceleration
    ADDR_GOAL_POSITION   = 42   # 目標位置 (mode=0/3)
    ADDR_GOAL_PWM        = 44   # Goal PWM  (mode=2) ※符号形式は実機で要確認
    ADDR_GOAL_VELOCITY   = 46   # Goal Velocity (mode=1/3)
    ADDR_TORQUE_LIMIT    = 48   # トルク制限 0〜1000

    ADDR_PRESENT_POSITION    = 56
    ADDR_PRESENT_VELOCITY    = 58
    ADDR_PRESENT_PWM         = 60
    ADDR_PRESENT_VOLTAGE     = 62
    ADDR_PRESENT_TEMPERATURE = 63
    ADDR_PRESENT_CURRENT     = 69

    # --- 動作モード定数 (addr 33) ---
    MODE_POSITION = 0
    MODE_VELOCITY = 1
    MODE_PWM      = 2
    MODE_STEP     = 3

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=1000000,
        timeout=0.03,
        mode=MODE_POSITION,
        ids=None,
        auto_scan=True,
        pwm_twos_complement=False,  # ★ Goal PWM の符号形式
                                    #   False = STS符号形式 (bit15=符号, 下位15bit=大きさ)
                                    #   True  = 2の補数形式
                                    #   マイナスで全速になる場合は True に切り替える
    ):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.ids  = []
        self.mode = mode
        self.pwm_twos_complement = pwm_twos_complement
        self._step_total = {}

        print(f"STSBus opened : {port}  pwm_encoding={'2comp' if pwm_twos_complement else 'STS'}")

        if ids is not None:
            self.ids = list(ids)
            print(f"Target IDs (manual): {self.ids}")
            self.set_mode_all(mode)
        elif auto_scan:
            self.scan_ids()
            self.set_mode_all(mode)

    # =========================================================
    # 通信プロトコル
    # =========================================================

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
    def _enc_signed_sts(value: int) -> int:
        """STS符号形式: bit15=符号, 下位15bit=大きさ"""
        value = int(value)
        return ((abs(value) & 0x7FFF) | 0x8000) if value < 0 else (value & 0x7FFF)

    @staticmethod
    def _enc_signed_twos(value: int) -> int:
        """2の補数形式(通常の符号付き16bit)"""
        value = int(value)
        return ((1 << 16) + value) if value < 0 else (value & 0xFFFF)

    def _enc_pwm(self, value: int) -> int:
        """
        Goal PWM (addr 44) 用の符号エンコード。
        実機の挙動によって切り替える:
            マイナスで全速 → インスタンス生成時に pwm_twos_complement=True を指定
        """
        value = max(-1000, min(1000, int(value)))
        if self.pwm_twos_complement:
            return self._enc_signed_twos(value)
        return self._enc_signed_sts(value)

    # =========================================================
    # PING / SCAN
    # =========================================================

    def ping(self, sid):
        packet = [0xFF, 0xFF, sid, 0x02, self.INST_PING]
        packet.append(self.checksum(packet[2:]))
        self.ser.reset_input_buffer()
        self.send(bytes(packet))
        resp = self.receive()
        return len(resp) > 0

    def scan_ids(self):
        print("\nScanning Servo IDs...\n")
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
        print(f"\nAvailable IDs: {self.ids}\n")
        return self.ids

    # =========================================================
    # READ
    # =========================================================

    def read(self, sid, address, size):
        packet = [0xFF, 0xFF, sid, 0x04, self.INST_READ, address, size]
        packet.append(self.checksum(packet[2:]))
        self.ser.reset_input_buffer()
        self.send(bytes(packet))
        resp = self.receive()
        return resp[5:-1] if len(resp) >= 7 else None

    def read_u8(self, sid, address):
        data = self.read(sid, address, 1)
        return data[0] if data else None

    def read_u16(self, sid, address):
        data = self.read(sid, address, 2)
        return (data[0] | data[1] << 8) if data and len(data) >= 2 else None

    def get_position(self, sid):
        return self.read_u16(sid, self.ADDR_PRESENT_POSITION)

    def get_mode(self, sid):
        return self.read_u8(sid, self.ADDR_MODE)

    # =========================================================
    # WRITE
    # =========================================================

    def write_u8(self, sid, address, value):
        packet = [0xFF, 0xFF, sid, 0x04, self.INST_WRITE,
                  address, value & 0xFF]
        packet.append(self.checksum(packet[2:]))
        self.send(bytes(packet))
        self.receive()

    def write_u16(self, sid, address, value):
        packet = [0xFF, 0xFF, sid, 0x05, self.INST_WRITE,
                  address, value & 0xFF, (value >> 8) & 0xFF]
        packet.append(self.checksum(packet[2:]))
        self.send(bytes(packet))
        self.receive()

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

    # =========================================================
    # 動作モード (EEPROM addr 33)
    # =========================================================

    def set_mode(self, sid, mode, force=False):
        if not force:
            current = self.get_mode(sid)
            if current is not None and current == mode:
                self.mode = mode
                return
        self.torque_off(sid)
        self.write_u8(sid, self.ADDR_LOCK, 0)
        self.write_u8(sid, self.ADDR_MODE, mode)
        self.write_u8(sid, self.ADDR_LOCK, 1)
        self.torque_on(sid)
        self.mode = mode
        print(f"ID={sid} MODE -> {mode}")

    def set_mode_all(self, mode, force=False):
        for sid in self.ids:
            self.set_mode(sid, mode, force=force)

    # =========================================================
    # トルク ON/OFF
    # =========================================================

    def torque_on(self, sid):
        self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 1)

    def torque_off(self, sid):
        self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 0)

    # =========================================================
    # PWM Mode 専用制御  (addr 33 = 2 = MODE_PWM を前提)
    # =========================================================

    def torque_free(self, servo_pwms: list[tuple[int, int]]) -> None:
        """
        複数サーボに PWM 値を同時に書き込む (SYNC WRITE)。
        停止は pwm=0 で再呼び出し。

        servo_pwms: [(sid, pwm), ...]
            pwm 正 → 正回転 / 負 → 逆回転 / 0 → 停止
            範囲: -1000 〜 1000

        ★ Torque Enable (addr 40) が 0 だと PWM を送っても動かない。
           set_mode() 内で torque_on() を呼んでいるため、set_mode(sid, MODE_PWM)
           を済ませてから呼ぶこと。

        ★ マイナスで全速になる場合は、インスタンス生成時に
           pwm_twos_complement=True を指定して符号形式を切り替える。
        """
        items = list(servo_pwms)
        servo_data = []
        for sid, pwm in items:
            enc = self._enc_pwm(pwm)
            servo_data.append((sid, [enc & 0xFF, (enc >> 8) & 0xFF]))
            print(f"  ID={sid}  PWM={pwm:+d}  encoded={enc:#06x}"
                  f"  bytes=[{enc&0xFF:02X} {(enc>>8)&0xFF:02X}]")
        self.sync_write(self.ADDR_GOAL_PWM, 2, servo_data)

    def torque_rotate_degrees(
        self,
        servo_pwms: list[tuple[int, int]],
        degrees: float,
        poll_interval: float = 0.01,
    ) -> None:
        """
        各サーボを指定 PWM で回転させ、スタート位置から degrees 以上
        回った個体を個別に停止する。全停止でリターン。

        servo_pwms: [(sid, pwm), ...]
        degrees   : 停止するまでの移動角度 (正値)
        poll_interval: ポーリング間隔(秒)

        移動量の積算:
            get_position() の 0〜4095 生値から差分を計算し、
            折り返し(0↔4095)を補正して累積する。
            累積値 × (360/4096) = 移動角度(度)
            pwm の符号で回転方向を決め、その方向の積算が
            degrees に達した時点で停止。

        ★ トルクが効いていない場合のチェックリスト:
           1. set_mode(sid, MODE_PWM) を先に呼んでいるか
           2. torque_on(sid) が呼ばれているか (set_mode が自動でONにする)
           3. pwm 値が小さすぎないか (100 未満は摩擦で動かない場合がある)
           4. マイナスで全速 → pwm_twos_complement=True を試す
        """
        items      = list(servo_pwms)
        ids        = [sid for sid, _ in items]
        pwm_map    = {sid: pwm for sid, pwm in items}
        direction  = {sid: (1 if pwm >= 0 else -1) for sid, pwm in items}

        # ── スタート位置を記録 ──────────────────────────────
        prev: dict[int, int]        = {}
        accumulated: dict[int, float] = {}
        for sid in ids:
            # Torque Enable を明示的に確認して ON にする
            self.torque_on(sid)
            pos = self.get_position(sid)
            if pos is None:
                raise RuntimeError(f"ID={sid} position read failed")
            prev[sid]        = pos
            accumulated[sid] = 0.0

        print(f"\ntorque_rotate_degrees  target={degrees}°")
        for sid, pwm in items:
            print(f"  ID={sid}  PWM={pwm:+d}  dir={'CW' if pwm>=0 else 'CCW'}")

        # ── 全サーボ同時スタート ────────────────────────────
        self.torque_free(items)

        active = set(ids)

        while active:
            time.sleep(poll_interval)
            stop_ids = []

            for sid in list(active):
                pos = self.get_position(sid)
                if pos is None:
                    continue

                # 折り返し補正
                diff = pos - prev[sid]
                if diff < -2048:
                    diff += 4096   # 4095→0 の正転折り返し
                elif diff > 2048:
                    diff -= 4096   # 0→4095 の逆転折り返し

                accumulated[sid] += diff * direction[sid]
                prev[sid] = pos

                moved_deg = accumulated[sid] * (360.0 / 4096.0)
                print(f"  ID={sid}  moved={moved_deg:.1f}° / {degrees}°")

                if moved_deg >= degrees:
                    stop_ids.append(sid)

            # 目標達成 ID を個別停止
            if stop_ids:
                self.torque_free([(sid, 0) for sid in stop_ids])
                for sid in stop_ids:
                    active.discard(sid)
                    print(f"  ID={sid}  STOP  ({accumulated[sid]*(360/4096):.1f}°)")

        print("torque_rotate_degrees complete\n")

    def torque_rotate_time(
        self,
        servo_pwms: list[tuple[int, int]],
        time_ms: int,
    ) -> None:
        """
        各サーボを指定 PWM で回転させ、time_ms ミリ秒後に停止する。

        servo_pwms: [(sid, pwm), ...]
        time_ms   : 回転時間 (ミリ秒)

        例:
            bus.torque_rotate_time([(1, 300), (2, -300)], 2000)
        """
        items = list(servo_pwms)

        # Torque Enable を明示的に ON にしてからスタート
        for sid, _ in items:
            self.torque_on(sid)

        print(f"\ntorque_rotate_time  {items}  {time_ms}ms")
        self.torque_free(items)

        time.sleep(time_ms / 1000)

        self.torque_free([(sid, 0) for sid, _ in items])
        print("torque_rotate_time complete\n")


# =========================================================
# エントリポイント
# =========================================================

def main():
    # ── 接続 ──────────────────────────────────────────────
    # マイナスで全速になる場合は pwm_twos_complement=True を試す
    bus = STSBus(
        port="/dev/ttyUSB0",
        ids=[1, 2],
        mode=STSBus.MODE_PWM,
        pwm_twos_complement=False,  # ← 暴走するなら True に変える
    )

    bus.torque_on(1)
    bus.torque_on(2)

    # ── テスト 1: 2秒回転して停止 ─────────────────────────
    bus.torque_rotate_time(
        [(1, 100), (2, -100)],
        500,
    )

    time.sleep(100.0)

    # ── テスト 2: 180度回転して停止 ───────────────────────
    bus.torque_rotate_degrees(
        [(1, 300), (2, -300)],
        degrees=180.0,
    )

    bus.torque_off(1)
    bus.torque_off(2)

    bus.close()


if __name__ == "__main__":
    
    main()
