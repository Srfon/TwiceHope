"""
sts3032_7.py
Mode2 (PWM/Torque) position-control wrapper for STS3032

[修正] DIR (方向反転) を送信PWMにも正しく適用
  - get_angle()  : (pos - home) * DIR  → 角度の向きを統一
  - PID update() : pwm *= DIR[sid]     → 誤差の符号に合わせてPWM方向を反転
  - on()         : pwm *= DIR[sid]     → 直接PWMにもDIR適用
  - move_for_time(): pwm *= DIR[sid]  → 時限PWMにもDIR適用
  これにより DIR={1:-1, 2:-1, ...} と設定すれば
  「正の角度指令 = 正の方向」が全IDで統一される
"""

import serial
import time
import threading

_TX_WAIT = 0.0002   # 送信後ハーフデュプレックス切替待ち [s]


class STS3032Bus:
    # ── アドレス ──────────────────────────────────────────
    ADDR_WORK_MODE           = 33
    ADDR_TORQUE_ENABLE       = 40
    ADDR_GOAL_PWM            = 44
    ADDR_PRESENT_POSITION    = 56
    ADDR_PRESENT_PWM         = 60
    ADDR_PRESENT_VOLTAGE     = 62
    ADDR_PRESENT_TEMPERATURE = 63

    MODE_TORQUE      = 2
    UPDATE_INTERVAL  = 1.0 / 50   # 50Hz PIDループ

    # ── 方向テーブル ──────────────────────────────────────
    # +1: そのまま  -1: 反転
    # ここを変えるだけで全経路(角度読み取り・PID出力・直接PWM)が一括反転する
    DIR = {
        1: -1,
        2: 1,
        3:  1,
        4:  1,
    }

    # ── 初期化 ────────────────────────────────────────────
    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000, timeout=0.05):
        self.ser   = serial.Serial(port, baudrate, timeout=timeout)
        self._lock = threading.Lock()

        self.ids          = []
        self.home         = {}   # {sid: raw_pos}
        self.target_angle = {}   # {sid: float | None}
        self.integral     = {}
        self.prev_error   = {}

        self.kp      = 6.0
        self.ki      = 0.0
        self.kd      = 0.15
        self.max_pwm = 1000

        self._max_pwm_override = {}   # {sid: int}
        self._time_deadline    = {}   # {sid: monotonic}

        self._running = False
        self._thread  = None

        self.scan_ids()

    # ── DIR ヘルパー ──────────────────────────────────────
    def _dir(self, sid: int) -> int:
        """IDの方向係数を返す (+1 or -1)"""
        return self.DIR.get(sid, 1)

    def _apply_dir_pwm(self, sid: int, pwm: int) -> int:
        """
        PWM値に方向係数を掛けて符号を整合させる。
        DIR=-1 のサーボは正の誤差に対して負のPWMを送らないと
        正方向に動かないため必須。
        """
        return int(pwm * self._dir(sid))

    # ── チェックサム ──────────────────────────────────────
    @staticmethod
    def _checksum(data):
        return (~sum(data)) & 0xFF

    # ── 生送受信 ──────────────────────────────────────────
    def _raw_send(self, pkt: bytes):
        self.ser.write(pkt)

    def _raw_recv(self, size: int) -> bytes:
        return self.ser.read(size)

    # ── WRITE u8 (レスポンスあり) ────────────────────────
    def _write_u8(self, sid, addr, value):
        pkt = [0xFF, 0xFF, sid, 0x04, 0x03, addr, value & 0xFF]
        pkt.append(self._checksum(pkt[2:]))
        with self._lock:
            self.ser.reset_input_buffer()
            self._raw_send(bytes(pkt))
            time.sleep(_TX_WAIT)
            self._raw_recv(6)

    # ── READ (ハーフデュプレックス安全版) ─────────────────
    def _read_raw(self, sid, addr, size) -> bytes | None:
        pkt = [0xFF, 0xFF, sid, 0x04, 0x02, addr, size]
        pkt.append(self._checksum(pkt[2:]))
        expected = 6 + size
        with self._lock:
            self.ser.reset_input_buffer()
            self._raw_send(bytes(pkt))
            time.sleep(_TX_WAIT)
            resp = self._raw_recv(expected)
        if len(resp) < expected:
            return None
        if resp[0] != 0xFF or resp[1] != 0xFF or resp[2] != sid:
            return None
        return resp[5: 5 + size]

    def _read_u8(self, sid, addr) -> int | None:
        d = self._read_raw(sid, addr, 1)
        return None if d is None else d[0]

    def _read_u16(self, sid, addr) -> int | None:
        d = self._read_raw(sid, addr, 2)
        if d is None:
            return None
        return d[0] | (d[1] << 8)   # リトルエンディアン

    # ── SYNC WRITE (ブロードキャスト・レスポンスなし) ────────
    def _sync_write_pwm(self, pwm_dict: dict):
        """
        {sid: raw_pwm_int, ...} を受け取り SYNC WRITE で一括送信。
        PWMは符号付き16bit → バイト変換は & 0xFFFF で正しく扱う。
        レスポンスなしなので receive() しない。
        """
        if not pwm_dict:
            return
        params = [self.ADDR_GOAL_PWM, 2]   # start_addr, data_len
        for sid, pwm in pwm_dict.items():
            pwm = max(-1000, min(1000, int(pwm)))
            params.append(sid)
            params.append(pwm & 0xFF)
            params.append((pwm >> 8) & 0xFF)
        length = len(params) + 2
        pkt = [0xFF, 0xFF, 0xFE, length, 0x83]
        pkt.extend(params)
        pkt.append(self._checksum(pkt[2:]))
        with self._lock:
            self._raw_send(bytes(pkt))
            time.sleep(_TX_WAIT)

    # ── スキャン / Ping ───────────────────────────────────
    def _ping(self, sid) -> bool:
        pkt = [0xFF, 0xFF, sid, 0x02, 0x01]
        pkt.append(self._checksum(pkt[2:]))
        with self._lock:
            self.ser.reset_input_buffer()
            self._raw_send(bytes(pkt))
            time.sleep(_TX_WAIT)
            resp = self._raw_recv(6)
        return len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF

    def scan_ids(self):
        self.ids.clear()
        for sid in range(1, 254):
            try:
                if self._ping(sid):
                    self.ids.append(sid)
                    print(f"FOUND ID={sid}  DIR={self._dir(sid):+d}")

                if sid > 4:
                    break
            except Exception:
                pass
        print("AVAILABLE IDS:", self.ids)

    # ── 有効化 / ゼロ点 ───────────────────────────────────
    def enable(self):
        for sid in self.ids:
            self._write_u8(sid, self.ADDR_WORK_MODE,     self.MODE_TORQUE)
            self._write_u8(sid, self.ADDR_TORQUE_ENABLE, 1)
        self._start_thread()
        print("Enabled. Background PID thread started.")

    def zero(self, sid):
        pos = self.get_position(sid)
        if pos is None:
            print(f"[zero] ID={sid} read failed, skipped.")
            return
        self.home[sid]         = pos
        self.target_angle[sid] = 0.0
        self.integral[sid]     = 0.0
        self.prev_error[sid]   = 0.0
        print(f"[zero] ID={sid}  raw_home={pos}  DIR={self._dir(sid):+d}")

    def zero_all(self):
        for sid in self.ids:
            self.zero(sid)

    # ── センサ読み取り ────────────────────────────────────
    def get_position(self, sid) -> int | None:
        return self._read_u16(sid, self.ADDR_PRESENT_POSITION)

    def get_voltage(self, sid) -> float | None:
        v = self._read_u8(sid, self.ADDR_PRESENT_VOLTAGE)
        return None if v is None else v / 10.0

    def get_temperature(self, sid) -> int | None:
        return self._read_u8(sid, self.ADDR_PRESENT_TEMPERATURE)

    def get_pwm(self, sid) -> int | None:
        raw = self._read_u16(sid, self.ADDR_PRESENT_PWM)
        if raw is None:
            return None
        return raw - 65536 if raw > 32767 else raw

    def get_angle(self, sid) -> float | None:
        """
        現在角度 [deg] を返す。DIR を掛けて論理方向に統一。
        DIR=-1 のサーボでも正方向コマンドで正の角度が返る。
        """
        pos = self.get_position(sid)
        if pos is None:
            return None
        home = self.home.get(sid, pos)
        return (pos - home) * (360.0 / 4096.0) * self._dir(sid)

    # ── 移動指令 ──────────────────────────────────────────
    def move(self, targets: dict):
        """絶対角度PID追従  {sid: deg, ...}"""
        for sid, deg in targets.items():
            self.target_angle[sid] = float(deg)
            self._time_deadline.pop(sid, None)

    def move_relative(self, targets: dict):
        """現在target_angleからの相対移動  {sid: deg, ...}"""
        for sid, deg in targets.items():
            base = self.target_angle.get(sid) or 0.0
            self.target_angle[sid] = base + float(deg)
            self._time_deadline.pop(sid, None)

    def move_for_time(self, targets: dict):
        """
        指定秒数だけ指定PWMをかけて自動停止。
        pwm は論理方向で指定 (DIR が自動適用される)。
        targets = {sid: {"pwm": 500, "time": 2.0}, ...}
        """
        now      = time.monotonic()
        pwm_dict = {}
        for sid, p in targets.items():
            pwm_logical = max(-1000, min(1000, int(p.get("pwm", 500))))
            sec         = float(p.get("time", 1.0))
            self._time_deadline[sid] = now + sec
            self.target_angle[sid]   = None   # PIDスキップ
            # ★ DIR適用: 論理PWM → 物理PWM
            pwm_dict[sid] = self._apply_dir_pwm(sid, pwm_logical)
        self._sync_write_pwm(pwm_dict)

    def move_for_deg(self, targets: dict):
        """
        現在位置から相対角度移動してPID停止。
        targets = {sid: {"deg": 90}, sid: {"deg": -45, "pwm": 600}, ...}
        """
        for sid, p in targets.items():
            deg = float(p.get("deg", 0.0))
            if "pwm" in p:
                self._max_pwm_override[sid] = int(p["pwm"])
            else:
                self._max_pwm_override.pop(sid, None)
            base = self.target_angle.get(sid)
            if base is None:
                base = self.get_angle(sid) or 0.0
            self.target_angle[sid] = base + deg
            self._time_deadline.pop(sid, None)

    # ── 直接PWM (論理方向で指定) ──────────────────────────
    def on(self, pwm_dict: dict):
        """
        論理PWMで直接制御  {sid: pwm, ...}
        DIR が自動適用されるので、正の値 = 正の論理方向。
        物理的な配線・取付方向の違いは DIR で吸収される。
        """
        physical = {
            sid: self._apply_dir_pwm(sid, pwm)
            for sid, pwm in pwm_dict.items()
        }
        self._sync_write_pwm(physical)

    def off(self):
        """全サーボPWM=0"""
        self._sync_write_pwm({sid: 0 for sid in self.ids})

    # ── PID update ────────────────────────────────────────
    def update(self):
        """
        1. 各IDの角度を逐次読み取り (論理角度 = DIR適用済み)
        2. PID演算 (論理空間で完結)
        3. pwm_logical に DIR を掛けて物理PWMへ変換
        4. 全サーボ SYNC WRITE 1回
        """
        now      = time.monotonic()
        pwm_dict = {}   # {sid: physical_pwm}

        for sid in self.ids:
            deadline = self._time_deadline.get(sid)

            # ── move_for_time タイムアウト ──
            if deadline is not None:
                if now >= deadline:
                    self._time_deadline.pop(sid, None)
                    angle = self.get_angle(sid) or 0.0
                    self.target_angle[sid] = angle
                    self.integral[sid]     = 0.0
                    self.prev_error[sid]   = 0.0
                    pwm_dict[sid] = 0
                # タイムアウト前はmove_for_timeが送信済み → スキップ
                continue

            target = self.target_angle.get(sid)
            if target is None:
                continue

            # 論理角度を読む (DIR適用済み)
            current = self.get_angle(sid)
            if current is None:
                print(f"[update] ID={sid} read failed, skip")
                continue

            # PID (論理空間 → 論理PWM)
            err                  = target - current
            self.integral[sid]   = self.integral.get(sid, 0.0) + err
            diff                 = err - self.prev_error.get(sid, 0.0)
            self.prev_error[sid] = err

            pwm_logical = (
                self.kp * err
                + self.ki * self.integral[sid]
                + self.kd * diff
            )

            limit = self._max_pwm_override.get(sid, self.max_pwm)

            if abs(err) < 1.0:
                pwm_logical = 0

            pwm_logical = max(-limit, min(limit, pwm_logical))

            # ★ DIR適用: 論理PWM → 物理PWM
            pwm_dict[sid] = self._apply_dir_pwm(sid, pwm_logical)

        # 全サーボまとめて SYNC WRITE
        if pwm_dict:
            self._sync_write_pwm(pwm_dict)

    # ── バックグラウンドスレッド ───────────────────────────
    def _start_thread(self):
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _stop_thread(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self):
        while self._running:
            t0 = time.monotonic()
            try:
                self.update()
            except Exception as e:
                print(f"[loop error] {e}")
            elapsed = time.monotonic() - t0
            rem = self.UPDATE_INTERVAL - elapsed
            if rem > 0:
                time.sleep(rem)

    # ── クローズ ──────────────────────────────────────────
    def close(self):
        self._stop_thread()
        self.off()
        self.ser.close()
        print("Closed.")


# ── 使用例 ────────────────────────────────────────────────
if __name__ == "__main__":
    motor = STS3032Bus("/dev/ttyUSB0")
    motor.enable()
    motor.zero_all()

    try:
        # 相対角度 (IDごとにmax_pwm制限も可)
        print("=== move_for_deg ===")
        motor.move_for_deg({
            1: {"deg":  400, "pwm": 700},
            2: {"deg":  400,"pwm": 700},
        })
        motor.off()
        motor.close()
        time.sleep(1000)

        # ゼロ戻し
        print("=== return to zero ===")
        motor.move({sid: 0 for sid in motor.ids})
        time.sleep(3)

    except KeyboardInterrupt:
        pass
    finally:
        motor.close()