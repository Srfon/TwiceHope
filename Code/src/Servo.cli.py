import serial
import time


class STSBus:
    """
    FEETECH STS3032 シリアルサーボ制御クラス。

    動作モード(WorkMode / レジスタ 33)を「引数1つ」で切り替えられる。
        mode=0 : サーボ(位置)モード   位置 0〜4095 を指定
        mode=1 : 連続回転・速度モード   速度 ±10〜9999 を指定(負で逆回転)
        mode=2 : 連続回転・トルクモード Time ±50〜1000 を指定(負で逆回転)
        mode=3 : ステップモード         移動量 ±30719 ステップを指定

    サーボモード(mode=0)は「相対(積算)」で動かす。
        move(sid, 1000) を呼ぶと内部目標が 0 -> 1000 になり、
        もう一度 move(sid, 1000) を呼ぶと 1000 -> 2000 になる
        (= 指定値の分だけ毎回追加で動く)。
        絶対値で動かしたいときは relative=False を渡す。
    """

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
        servo_relative=True,
        servo_wrap=False,
        left_ids=None,
        right_ids=None,
        directions=None,
    ):
        """
        mode            : 起動時に全サーボへ設定する動作モード(引数1つで切替)
        ids             : 制御するサーボIDのリスト。自分で指定したいときに使う。
                          例) ids=[1, 3, 4]
                          指定すると自動スキャンせず、このIDだけを動かす。
                          None のときは auto_scan の設定に従う。
        auto_scan       : ids 未指定のとき、接続中のサーボを自動探索するか
        servo_relative  : サーボモードを相対(積算)で動かすか
        servo_wrap      : サーボモードで 4095 を超えたとき 0 に巻き戻して
                          周回し続けるか。False なら 4095 で頭打ち(警告表示)。
                          ※ 連続して回したいなら mode=3(ステップ)推奨。
        """
        self.ser = serial.Serial(
            port,
            baudrate,
            timeout=timeout,
        )

        self.ids = []
        self.mode = mode
        self.servo_relative = servo_relative
        self.servo_wrap = servo_wrap

        # --- 進行方向(左右)の設定 -----------------------------------
        # 左右のサーボは鏡像配置のため、同じ「前進」でも左は + ・右は -
        # の物理指令が必要になる。各サーボに向き(+1/-1)を持たせ、
        # 論理値(ロボット前進 = 正)に掛けて物理指令へ変換する。
        #   left_ids  に入れたID  -> +1
        #   right_ids に入れたID  -> -1
        #   directions={1:+1, 3:-1, ...} のように直接指定も可(left/right より優先)
        self._left = set(left_ids or [])
        self._right = set(right_ids or [])
        if directions is not None:
            self.directions = dict(directions)
        else:
            self.directions = {}
            for sid in self._left:
                self.directions[sid] = +1
            for sid in self._right:
                self.directions[sid] = -1

        # サーボモードの相対動作で使う「内部目標位置」。
        # 起動直後は 0 から始まるので、最初の move(sid, 1000) で 1000、
        # 次の move(sid, 1000) で 2000 になる。
        self._logical = {}

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
    # 進行方向(左右の向き)
    # =====================================================

    def _dir(self, sid):
        """サーボの向き(+1 / -1)。左 + ・右 - 。未設定は +1。"""
        return self.directions.get(sid, 1)

    def _apply_dir(self, sid, value):
        """論理値(ロボット前進 = 正)を、そのサーボの物理指令へ変換する。"""
        return value * self._dir(sid)

    def _turn_factor(self, sid):
        """旋回の係数。左 = +1 ・右 = -1(ロボット幾何上の左右)。"""
        if sid in self._right:
            return -1
        return +1

    def set_direction(self, sid, sign):
        """サーボの向きを後から設定する。sign は +1 か -1。"""
        self.directions[sid] = 1 if sign >= 0 else -1

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
        self._logical.pop(sid, None)               # 相対の積算値をリセット

        print(f"ID={sid} MODE -> {mode}")

    def set_mode_all(self, mode, force=False):
        for sid in self.ids:
            self.set_mode(sid, mode, force=force)

    # =====================================================
    # 相対動作の内部目標位置の管理(サーボモード用)
    # =====================================================

    def set_origin(self, sid, value=0):
        """サーボモードの相対動作の基準(内部目標位置)を設定する。"""
        self._logical[sid] = value

    def reset_origin(self, value=0):
        for sid in self.ids:
            self._logical[sid] = value

    # =====================================================
    # 個別制御 (モードで動作が変わる)
    # =====================================================

    def move(
        self,
        sid,
        value,
        speed=0,
        time_ms=0,
        mode=None,
        relative=None,
        repeat=1,
        interval=0.0,
    ):
        """
        value の意味は動作モードで変わる:
            mode=0 サーボ : 目標位置(相対なら追加分)
            mode=1 速度   : 回転速度 ±10〜9999
            mode=2 トルク : Time ±50〜1000
            mode=3 ステップ: 移動量 ±30719

        repeat   : 同じコマンドを何回送るか(相対サーボ/ステップなら回数分積算)
        interval : 1回ごとの待ち時間(秒)
        """
        m = self.mode if mode is None else mode
        n = max(1, int(repeat))

        for i in range(n):
            if m == self.MODE_SERVO:
                self._move_servo(sid, value, speed, time_ms, relative)
            elif m == self.MODE_SPEED:
                self._move_speed(sid, value)
            elif m == self.MODE_TORQUE:
                self._move_torque(sid, value)
            elif m == self.MODE_STEP:
                self._move_step(sid, value, speed)
            else:
                raise ValueError(f"unknown mode: {m}")

            if interval and i < n - 1:
                time.sleep(interval)

    def _servo_target(self, sid, value, relative):
        """サーボモードの目標位置を計算(相対積算 + 4096超えの処理)。"""
        rel = self.servo_relative if relative is None else relative

        if rel:
            # 相対の増分にだけ向きを掛ける(左 + ・右 -)
            raw = self._logical.get(sid, 0) + self._apply_dir(sid, value)
        else:
            raw = value

        if self.servo_wrap:
            # 0〜4095 で巻き戻して周回し続ける
            target = raw % 4096
        else:
            target = max(0, min(4095, raw))
            if raw != target:
                print(
                    f"!! ID={sid} サーボモードは1回転(0〜4095)が上限です。"
                    f"目標 {raw} は {target} に制限されました。"
                    f"このまま相対指令を続けても動きません。"
                    f"連続回転したいときは mode=3(ステップ)を使うか、"
                    f"servo_wrap=True にしてください。"
                )

        self._logical[sid] = target
        return target, rel

    def _move_servo(self, sid, value, speed, time_ms, relative):
        target, rel = self._servo_target(sid, value, relative)

        speed = max(0, min(4095, speed))

        self.write_u16(sid, self.ADDR_MOVE_TIME, time_ms)
        self.write_u16(sid, self.ADDR_MOVE_SPEED, speed)
        self.write_u16(sid, self.ADDR_GOAL_POSITION, target)

        print(f"ID={sid} [SERVO] GOAL={target} SPEED={speed} (rel={rel})")

    def _move_speed(self, sid, speed):
        # 連続回転・速度モード。負の値で逆回転。
        phys = self._apply_dir(sid, speed)
        self.write_u16(sid, self.ADDR_MOVE_SPEED, self._enc_signed(phys))
        print(f"ID={sid} [SPEED] SPEED={speed} -> 物理 {phys}")

    def _move_torque(self, sid, value):
        # 連続回転・トルクモード。負の値で逆回転。
        # ※ 記事の実測ではトルク指定が addr 41 に出ていた。実機で要確認。
        phys = self._apply_dir(sid, value)
        self.write_u16(sid, self.ADDR_GOAL_ACC, self._enc_signed(phys))
        print(f"ID={sid} [TORQUE] TIME={value} -> 物理 {phys}")

    def _move_step(self, sid, steps, speed):
        # ステップモード。1回ごとに steps だけ相対回転する(ハードウェア相対)。
        # 多回転対応なので 4096 では止まらないが、累計の絶対位置は ±30719 まで。
        steps = self._apply_dir(sid, int(steps))   # 前進 = 正 を物理方向へ
        total = self._step_total.get(sid, 0)
        new_total = total + steps

        if not (-30719 <= new_total <= 30719):
            print(
                f"!! ID={sid} ステップモードの可動範囲(±30719 ≈ 約7.5回転)を"
                f"超えます(累計 {new_total})。これ以上は回りません。"
            )
            # 範囲内に収まる分だけ動かす
            clamped_total = max(-30719, min(30719, new_total))
            steps = clamped_total - total
            new_total = clamped_total

        self._step_total[sid] = new_total

        steps = max(-30719, min(30719, steps))
        speed = max(0, min(4095, speed))
        self.write_u16(sid, self.ADDR_MOVE_SPEED, speed)
        self.write_u16(sid, self.ADDR_GOAL_POSITION, self._enc_signed(steps))
        print(f"ID={sid} [STEP] STEPS={steps} 累計={new_total} SPEED={speed}")

    # =====================================================
    # SYNC WRITE
    # =====================================================

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
    # 複数同時制御 (モードで動作が変わる)
    # =====================================================

    def move_group(self, targets, mode=None, relative=None,
                   repeat=1, interval=0.0):
        """
        例:
            bus.move_group({
                1: {"position": 1000, "speed": 200},
                2: {"position": 1000, "speed": 200},
            }, repeat=3, interval=2)

        各モードでの value のキー:
            mode=0 サーボ : "position"(相対なら追加分), "speed", "time"
            mode=1 速度   : "speed"  (±10〜9999)
            mode=2 トルク : "torque" (±50〜1000)
            mode=3 ステップ: "position"(移動量 ±30719), "speed"

        repeat   : 同じコマンドを何回送るか(相対サーボ/ステップなら回数分積算)
        interval : 1回ごとの待ち時間(秒)
        """
        n = max(1, int(repeat))
        for i in range(n):
            self._move_group_once(targets, mode, relative)
            if interval and i < n - 1:
                time.sleep(interval)

    def _move_group_once(self, targets, mode=None, relative=None):
        m = self.mode if mode is None else mode

        print()
        print(f"SYNC MOVE (mode={m})")
        print()

        servo_data = []

        if m == self.MODE_SPEED:
            for sid, cfg in targets.items():
                phys = self._apply_dir(sid, cfg.get("speed", 0))
                spd = self._enc_signed(phys)
                servo_data.append(
                    (sid, [spd & 0xFF, (spd >> 8) & 0xFF])
                )
                print(f"ID={sid} [SPEED] SPEED={cfg.get('speed', 0)} -> 物理 {phys}")
            self.sync_write(self.ADDR_MOVE_SPEED, 2, servo_data)
            return

        if m == self.MODE_TORQUE:
            for sid, cfg in targets.items():
                phys = self._apply_dir(sid, cfg.get("torque", 0))
                trq = self._enc_signed(phys)
                servo_data.append(
                    (sid, [trq & 0xFF, (trq >> 8) & 0xFF])
                )
                print(f"ID={sid} [TORQUE] TIME={cfg.get('torque', 0)} -> 物理 {phys}")
            self.sync_write(self.ADDR_GOAL_ACC, 2, servo_data)
            return

        # ここから mode=0(サーボ) と mode=3(ステップ) は addr 42 に 6byte 書込み
        for sid, cfg in targets.items():
            spd = max(0, min(4095, cfg.get("speed", 0)))
            movetime = cfg.get("time", 0)
            pos_in = cfg.get("position", 0)

            if m == self.MODE_STEP:
                # 累計が ±30719 を超えないように調整(_move_step と同じ処理)
                step_in = self._apply_dir(sid, int(pos_in))   # 前進 = 正 を物理方向へ
                total = self._step_total.get(sid, 0)
                new_total = total + step_in
                if not (-30719 <= new_total <= 30719):
                    print(
                        f"!! ID={sid} ステップ可動範囲(±30719)を超えます"
                        f"(累計 {new_total})。これ以上は回りません。"
                    )
                    new_total = max(-30719, min(30719, new_total))
                step = new_total - total
                self._step_total[sid] = new_total
                pos = self._enc_signed(step)
                shown = f"step={step} 累計={new_total}"
            else:  # MODE_SERVO
                target, _ = self._servo_target(sid, pos_in, relative)
                pos = target
                shown = f"GOAL={target}"

            data = [
                pos & 0xFF, (pos >> 8) & 0xFF,
                movetime & 0xFF, (movetime >> 8) & 0xFF,
                spd & 0xFF, (spd >> 8) & 0xFF,
            ]
            servo_data.append((sid, data))

            tag = "STEP" if m == self.MODE_STEP else "SERVO"
            print(f"ID={sid} [{tag}] {shown} SPEED={spd} TIME={movetime}")

        self.sync_write(self.ADDR_GOAL_POSITION, 6, servo_data)

    # =====================================================
    # 走行(前進・後進・旋回) — 左右の向きを自動適用
    # =====================================================

    def drive(self, forward=0, turn=0, speed=200,
              repeat=1, interval=0.0, mode=None):
        """
        ロボットの動きを「前進量」「旋回量」で直接指示する高レベル関数。
        左右が鏡像配置でも、向き設定(左 + / 右 -)を自動で適用するので、
        forward を正にすれば必ず前へ進む。

            forward : 前進量(正 = 前進, 負 = 後進)
            turn    : 旋回量(正 = 右回り, 負 = 左回り)。その場旋回は forward=0
            speed   : mode 0/3 の動く速さ
            mode    : 動作モード。省略時は現在のモード。
                      ・連続走行 -> mode=1(速度)
                      ・距離指定 -> mode=3(ステップ)
                      ・角度制御 -> mode=0(サーボ ※可動は1回転以内)

        各サーボの論理値 = forward + turn * 旋回係数(左 +1 / 右 -1)。
        物理指令へは move_group 側で向き(左 + / 右 -)を掛けて変換する。
        """
        m = self.mode if mode is None else mode

        targets = {}
        for sid in self.ids:
            logical = forward + turn * self._turn_factor(sid)
            if m == self.MODE_SPEED:
                targets[sid] = {"speed": logical}
            elif m == self.MODE_TORQUE:
                targets[sid] = {"torque": logical}
            else:  # MODE_SERVO / MODE_STEP
                targets[sid] = {"position": logical, "speed": speed}

        self.move_group(targets, mode=m, repeat=repeat, interval=interval)

    def forward(self, value, **kw):
        """前進。value だけ前へ。"""
        self.drive(forward=abs(value), **kw)

    def backward(self, value, **kw):
        """後進。value だけ後ろへ。"""
        self.drive(forward=-abs(value), **kw)

    def spin_right(self, value, **kw):
        """その場で右回り。"""
        self.drive(forward=0, turn=abs(value), **kw)

    def spin_left(self, value, **kw):
        """その場で左回り。"""
        self.drive(forward=0, turn=-abs(value), **kw)

    def stop(self):
        """全サーボ停止。速度/トルクモードは 0 を送信、それ以外はトルクOFF。"""
        if self.mode == self.MODE_SPEED:
            self.move_group({sid: {"speed": 0} for sid in self.ids})
        elif self.mode == self.MODE_TORQUE:
            self.move_group({sid: {"torque": 0} for sid in self.ids})
        else:
            for sid in self.ids:
                self.torque_off(sid)

    # =====================================================
    # トルク
    # =====================================================

    def torque_on(self, sid):
        self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 1)

    def torque_off(self, sid):
        self.write_u8(sid, self.ADDR_TORQUE_ENABLE, 0)


# =========================================================
# ★ 直接実行(VSCodeの「実行」ボタン等)用の設定 ★
#   ターミナルで引数を打たずに動かす場合は、ここを書き換える。
#   (ターミナルから引数を渡したときは、そちらが優先されます)
# ---------------------------------------------------------
MODE = 0          # 0:サーボ 1:速度 2:トルク 3:ステップ
REPEAT = 3        # 何回動かすか
AMOUNT = 1000     # 1回あたりの移動量/速度/トルク
SPEED = 200       # 速度 (mode 0/3 用)
INTERVAL = 2.0    # 1回ごとの待ち時間(秒)
IDS = None        # 動かすID。None なら自動スキャンの全ID。例: [1, 2, 3]
PORT = "/dev/ttyUSB0"
RELATIVE = True   # サーボモードを相対(積算)にするか
WRAP = False      # サーボモードで 4095 を超えたら 0 に巻き戻して周回し続けるか
                  # (連続回転は mode=3 ステップ推奨。これは応急処置)
LEFT_IDS = None   # 左側サーボID 例:[1, 2] (前進で + 方向に回す)
RIGHT_IDS = None  # 右側サーボID 例:[3, 4] (前進で - 方向に回す)
# =========================================================


def run(
    mode=MODE,
    repeat=REPEAT,
    amount=AMOUNT,
    speed=SPEED,
    interval=INTERVAL,
    ids=IDS,
    port=PORT,
    relative=RELATIVE,
    wrap=WRAP,
    left_ids=LEFT_IDS,
    right_ids=RIGHT_IDS,
):
    """設定値で実際に動かす本体。コードから直接呼び出せる。"""
    bus = STSBus(port, mode=mode, servo_relative=relative, servo_wrap=wrap,
                 left_ids=left_ids, right_ids=right_ids)

    target_ids = ids if ids else bus.ids
    if not target_ids:
        print("対象サーボがありません。IDS で指定してください。")
        bus.close()
        return

    if mode == STSBus.MODE_SERVO:
        targets = {sid: {"position": amount, "speed": speed}
                   for sid in target_ids}
        bus.move_group(targets, repeat=repeat, interval=interval)

    elif mode == STSBus.MODE_STEP:
        targets = {sid: {"position": amount, "speed": speed}
                   for sid in target_ids}
        bus.move_group(targets, repeat=repeat, interval=interval)

    elif mode == STSBus.MODE_SPEED:
        bus.move_group({sid: {"speed": amount} for sid in target_ids},
                       repeat=repeat, interval=interval)

    elif mode == STSBus.MODE_TORQUE:
        bus.move_group({sid: {"torque": amount} for sid in target_ids},
                       repeat=repeat, interval=interval)

    bus.close()


def servo_move(
    mode,
    repeat=1,
    amount=AMOUNT,
    speed=SPEED,
    interval=INTERVAL,
    ids=IDS,
    port=PORT,
    relative=RELATIVE,
    wrap=WRAP,
    left_ids=LEFT_IDS,
    right_ids=RIGHT_IDS,
):
    """
    コードから1行で動かすための関数。

        servo_move(0, 3)                  # mode=0(サーボ)を3回。4095で頭打ち
        servo_move(3, 3, amount=1000)     # ステップmode。4096を超えても回り続ける
        servo_move(0, 5, amount=500)      # 移動量を変える
        servo_move(1, 1, amount=2000)     # 速度モード
        servo_move(0, 3, ids=[1, 2])      # ID指定
        servo_move(0, 10, wrap=True)      # サーボのまま 0〜4095 を周回し続ける
        servo_move(1, 1, 2000, left_ids=[1,2], right_ids=[3,4])  # 左右指定で前進
    """
    run(
        mode=mode,
        repeat=repeat,
        amount=amount,
        speed=speed,
        interval=interval,
        ids=ids,
        port=port,
        relative=relative,
        wrap=wrap,
        left_ids=left_ids,
        right_ids=right_ids,
    )


# =========================================================
# コマンドライン実行  (引数: モード 回数 [移動量] [速度])
#   例:
#     python servo_cli.py 0 3              # mode=0 を 3回
#     python servo_cli.py 0 5 500 300      # mode=0, 5回, 移動量500, 速度300
#     python servo_cli.py 3 2 4096         # ステップmode, 2回, 4096
#     python servo_cli.py 0 3 --ids 1 2    # ID指定
# =========================================================
def main():
    import argparse
    import sys

    # 引数なしで実行(VSCodeのRunボタン等) → 上の設定値で動かす
    if len(sys.argv) == 1:
        run()
        return

    parser = argparse.ArgumentParser(
        description="STS3032 サーボ制御 (モード・回数を引数で指定)",
        epilog="例: python servo_cli.py 0 3        (mode=0 を 3回)\n"
               "    python servo_cli.py 0 3 1000 200",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("mode", type=int, choices=[0, 1, 2, 3],
                        help="動作モード 0:サーボ 1:速度 2:トルク 3:ステップ")
    parser.add_argument("repeat", type=int, nargs="?", default=REPEAT,
                        help=f"何回動かすか (既定: {REPEAT})")
    parser.add_argument("amount", type=int, nargs="?", default=AMOUNT,
                        help=f"1回あたりの移動量/速度/トルク (既定: {AMOUNT})")
    parser.add_argument("speed", type=int, nargs="?", default=SPEED,
                        help=f"速度 mode 0/3 用 (既定: {SPEED})")
    parser.add_argument("--port", default=PORT,
                        help=f"シリアルポート (既定: {PORT})")
    parser.add_argument("--interval", type=float, default=INTERVAL,
                        help=f"1回ごとの待ち時間 秒 (既定: {INTERVAL})")
    parser.add_argument("--ids", type=int, nargs="*", default=IDS,
                        help="動かすID (省略時は自動スキャンの全ID)")
    parser.add_argument("--left", type=int, nargs="*", default=LEFT_IDS,
                        help="左側サーボID (前進で + 方向)")
    parser.add_argument("--right", type=int, nargs="*", default=RIGHT_IDS,
                        help="右側サーボID (前進で - 方向)")
    parser.add_argument("--no-relative", action="store_true",
                        help="サーボモードを絶対値で動かす")
    parser.add_argument("--wrap", action="store_true",
                        help="サーボモードで4095を超えたら0に巻き戻して周回する")

    args = parser.parse_args()

    run(
        mode=args.mode,
        repeat=args.repeat,
        amount=args.amount,
        speed=args.speed,
        interval=args.interval,
        ids=args.ids,
        port=args.port,
        relative=not args.no_relative,
        wrap=args.wrap,
        left_ids=args.left,
        right_ids=args.right,
    )


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        # ターミナルから引数で実行  例: python servo_cli.py 0 3
        main()
    else:
        # =================================================
        # ★ コードで直接動かす(VSCodeの実行ボタン等) ★
        #     servo_move(モード, 何度) と書くだけ
        # =================================================
        servo_move(0, 3)

        # 他の例:
        # servo_move(0, 5, amount=500, speed=300)
        # servo_move(3, 2, amount=4096)     # ステップ
        # servo_move(1, 1, amount=2000)     # 速度
        # servo_move(0, 3, ids=[1, 2])      # ID指定

        # --- 左右の向きを使った前進(差動駆動) --------------------
        # 左を ID 1,2 / 右を ID 3,4 とした例。
        # 速度モードで前へ進む(左 +2000 / 右 -2000 が自動で送られる):
        #   servo_move(1, 1, 2000, left_ids=[1, 2], right_ids=[3, 4])
        #
        # もっと直感的に書きたいときは STSBus を直接使う:
        #   bus = STSBus("/dev/ttyUSB0", mode=STSBus.MODE_SPEED,
        #                left_ids=[1, 2], right_ids=[3, 4])
        #   bus.forward(2000)      # 前進
        #   time.sleep(2)
        #   bus.spin_right(1500)   # その場で右回り
        #   time.sleep(1)
        #   bus.stop()             # 停止
        #   bus.close()