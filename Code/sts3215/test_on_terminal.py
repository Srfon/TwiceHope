#!/usr/bin/env python3
"""
STS3215 キーボード操作テストツール
=====================================================================

矢印キーでリアルタイムに「速度」または「目標角度」を増減させながら、
set_mode() が対応する全モード
    MODE_POSITION (0)
    MODE_VELOCITY (1)
    MODE_PWM      (2)
    MODE_STEP     (3)
を順番に切り替えてテストできるツール。

---------------------------------------------------------------------
操作方法
---------------------------------------------------------------------
    m         : モード切替 (POSITION -> VELOCITY -> PWM -> STEP -> ...)
                ※ set_mode() は内部で torque OFF する仕様のため、
                   モード切替直後は自動的に Torque = OFF になる。
                   動かしたい場合は t キーで Torque ON にすること。
    t         : Torque ON/OFF トグル
    ← / →    : 目標値を step 分だけ増減し、即座にサーボへ送信する
    ↑ / ↓    : 増減幅 (step) を変更する
    r         : 目標値をサーボの現在値で読み直して同期する
    SPACE     : 現在の目標値をもう一度送信する (再送)
    q / ESC   : Torque OFF にしてシリアルポートを閉じ、終了する

---------------------------------------------------------------------
モードごとの「目標値」の扱い
---------------------------------------------------------------------
    POSITION (0) : 目標値 = 目標角度  0 - 4095
                   -> servo.set_position() で ADDR_GOAL_POSITION (42) に書き込み

    VELOCITY (1) : 目標値 = 符号付き速度  -1023 ~ +1023
                   -> ADDR_GOAL_SPEED (46) に書き込み。
                      STS3215 の速度レジスタは bit15 を回転方向ビットとして
                      使用するため (bit15=0:正転 / bit15=1:逆転、bit0-14が
                      速度の絶対値)、send側でsigned_to_raw()により変換する。

    PWM      (2) : 目標値 = 符号付きデューティ比  -1023 ~ +1023
                   -> sts3215.py には PWM 専用レジスタ/setter が存在しないため、
                      VELOCITY と同じ ADDR_GOAL_SPEED (46) にデューティ比として
                      書き込む (bit15を方向/符号ビットとして流用)。

    STEP     (3) : 目標値 = 目標角度  0 - 4095
                   -> STEP モード専用のレジスタもドライバに実装されていないため、
                      暫定的に POSITION と同じ ADDR_GOAL_POSITION (42) を流用する。
                      実機の挙動がPOSITIONモードと異なる場合は、実際のレジスタ
                      マップに合わせて send_target() 内の分岐を調整すること。

---------------------------------------------------------------------
実装上の注意
---------------------------------------------------------------------
sts3215.py の read()/write() は通信内容をデバッグ用に print() で標準出力に
書き出す。この出力が curses の画面描画とバッティングして表示が崩れるため、
本ツールでは curses セッション中は sys.stdout を "sts3215_debug.log" に
リダイレクトしている（curses自体はttyへ直接描画するため影響を受けない）。
通信ログを見たい場合はこのログファイルを参照すること。
"""

import contextlib
import curses
import time

from sts3215 import STS3215


# =====================================================================
# 設定
# =====================================================================

PORT = "/dev/ttyUSB0"
SERVO_ID = 3
BAUDRATE = 1_000_000

DEBUG_LOG_PATH = "sts3215_debug.log"

POSITION_MIN = 0
POSITION_MAX = 4095
POSITION_STEP_DEFAULT = 10
POSITION_STEP_MIN = 1
POSITION_STEP_MAX = 200

SPEED_MIN = -1023
SPEED_MAX = 1023
SPEED_STEP_DEFAULT = 10
SPEED_STEP_MIN = 1
SPEED_STEP_MAX = 200

STATUS_POLL_INTERVAL = 0.5  # [s] 現在値をサーボへ読みに行く間隔
LOOP_TICK = 0.02            # [s] メインループの最小周期


# =====================================================================
# モード定義
# =====================================================================

MODE_SEQUENCE = [
    STS3215.MODE_POSITION,
    STS3215.MODE_VELOCITY,
    STS3215.MODE_PWM,
    STS3215.MODE_STEP,
]

MODE_NAMES = {
    STS3215.MODE_POSITION: "POSITION",
    STS3215.MODE_VELOCITY: "VELOCITY",
    STS3215.MODE_PWM: "PWM",
    STS3215.MODE_STEP: "STEP",
}


# =====================================================================
# 符号付き速度/デューティ比 <-> レジスタ生値 変換
#
# bit15 = 0 : 正転 (+), bit0-14 = 絶対値
# bit15 = 1 : 逆転 (-), bit0-14 = 絶対値
# =====================================================================

def signed_to_raw(value):

    magnitude = abs(value)
    magnitude = max(0, min(0x7FFF, magnitude))

    raw = magnitude

    if value < 0:
        raw |= 0x8000

    return raw


def raw_to_signed(raw):

    magnitude = raw & 0x7FFF
    sign = -1 if (raw & 0x8000) else 1

    return sign * magnitude


# =====================================================================
# アプリ状態
# =====================================================================

class AppState:

    def __init__(self, servo):

        self.servo = servo

        self.mode_index = 0
        self.mode = MODE_SEQUENCE[self.mode_index]

        self.target_position = 0
        self.target_speed = 0

        self.position_step = POSITION_STEP_DEFAULT
        self.speed_step = SPEED_STEP_DEFAULT

        self.torque_on = False

        self.present_position = None
        self.present_speed = None
        self.present_load = None
        self.moving = None

        self.last_error = ""
        self.last_status_poll = 0.0

    # -----------------------------------------------------------
    # モード種別判定
    # -----------------------------------------------------------

    def is_position_like(self):
        return self.mode in (
            STS3215.MODE_POSITION,
            STS3215.MODE_STEP,
        )

    def is_speed_like(self):
        return self.mode in (
            STS3215.MODE_VELOCITY,
            STS3215.MODE_PWM,
        )

    # -----------------------------------------------------------
    # 起動時: サーボの現在モードを読み取り、シーケンスに同期する
    # -----------------------------------------------------------

    def init_mode_from_servo(self):

        try:
            current = self.servo.get_mode(SERVO_ID)

            if current in MODE_SEQUENCE:
                self.mode_index = MODE_SEQUENCE.index(current)
                self.mode = current

        except Exception as e:
            self.last_error = f"init_mode error: {e}"

        self.sync_target_from_present()

    # -----------------------------------------------------------
    # モード切替
    # -----------------------------------------------------------

    def next_mode(self):

        self.mode_index = (self.mode_index + 1) % len(MODE_SEQUENCE)
        self.mode = MODE_SEQUENCE[self.mode_index]

        try:
            # set_mode() 内部で torque(False) が呼ばれる
            self.servo.set_mode(SERVO_ID, self.mode)
            self.last_error = ""

        except Exception as e:
            self.last_error = f"set_mode error: {e}"

        self.torque_on = False

        self.sync_target_from_present()

    # -----------------------------------------------------------
    # 目標値をサーボの現在値で初期化しなおす
    # -----------------------------------------------------------

    def sync_target_from_present(self):

        try:
            if self.is_position_like():
                self.target_position = self.servo.get_position(SERVO_ID)
            else:
                raw = self.servo.read_u16(
                    SERVO_ID,
                    self.servo.ADDR_GOAL_SPEED,
                )
                self.target_speed = raw_to_signed(raw)

            self.last_error = ""

        except Exception as e:
            self.last_error = f"sync error: {e}"

    # -----------------------------------------------------------
    # トルク ON/OFF トグル
    # -----------------------------------------------------------

    def toggle_torque(self):

        try:
            self.servo.torque(SERVO_ID, not self.torque_on)
            self.torque_on = not self.torque_on
            self.last_error = ""

        except Exception as e:
            self.last_error = f"torque error: {e}"

    # -----------------------------------------------------------
    # 目標値の増減 (即座に送信)
    # -----------------------------------------------------------

    def adjust_target(self, direction):

        if self.is_position_like():
            self.target_position += direction * self.position_step
            self.target_position = max(
                POSITION_MIN,
                min(POSITION_MAX, self.target_position),
            )
        else:
            self.target_speed += direction * self.speed_step
            self.target_speed = max(
                SPEED_MIN,
                min(SPEED_MAX, self.target_speed),
            )

        self.send_target()

    # -----------------------------------------------------------
    # 増減幅 (step) の変更
    # -----------------------------------------------------------

    def adjust_step(self, direction):

        if self.is_position_like():
            self.position_step += direction
            self.position_step = max(
                POSITION_STEP_MIN,
                min(POSITION_STEP_MAX, self.position_step),
            )
        else:
            self.speed_step += direction
            self.speed_step = max(
                SPEED_STEP_MIN,
                min(SPEED_STEP_MAX, self.speed_step),
            )

    # -----------------------------------------------------------
    # 現在の目標値をサーボへ送信
    # -----------------------------------------------------------

    def send_target(self):

        try:
            if self.is_position_like():
                # POSITION / STEP -> GOAL_POSITION
                self.servo.set_position(SERVO_ID, self.target_position)

            else:
                # VELOCITY / PWM -> GOAL_SPEED (符号はbit15)
                raw = signed_to_raw(self.target_speed)

                self.servo.write_u16(
                    SERVO_ID,
                    self.servo.ADDR_GOAL_SPEED,
                    raw,
                )

            self.last_error = ""

        except Exception as e:
            self.last_error = f"send error: {e}"

    # -----------------------------------------------------------
    # 現在状態のポーリング (一定間隔でのみ実行)
    # -----------------------------------------------------------

    def poll_status(self, force=False):

        now = time.time()

        if not force and (now - self.last_status_poll < STATUS_POLL_INTERVAL):
            return

        self.last_status_poll = now

        try:
            self.present_position = self.servo.get_position(SERVO_ID)
            self.present_speed = self.servo.get_speed(SERVO_ID)
            self.present_load = self.servo.get_load(SERVO_ID)
            self.moving = self.servo.is_moving(SERVO_ID)

            self.last_error = ""

        except Exception as e:
            self.last_error = f"poll error: {e}"


# =====================================================================
# 画面描画
# =====================================================================

def safe_addstr(stdscr, y, x, text, attr=0):
    """画面端でcurses.errorが出るのを無視して描画する"""

    try:
        stdscr.addstr(y, x, text, attr)
    except curses.error:
        pass


def draw(stdscr, state):

    stdscr.erase()

    height, width = stdscr.getmaxyx()

    bold = curses.A_BOLD
    reverse = curses.A_REVERSE

    row = 0

    safe_addstr(stdscr, row, 0, "STS3215 KEYBOARD CONTROL TEST", bold)
    row += 1
    safe_addstr(stdscr, row, 0, "=" * min(60, width - 1))
    row += 2

    # ---- モード ----
    mode_name = MODE_NAMES.get(state.mode, str(state.mode))
    safe_addstr(
        stdscr, row, 0,
        f"MODE   : {mode_name} (index={state.mode})",
        bold,
    )
    row += 1

    torque_str = "ON" if state.torque_on else "OFF"
    torque_attr = reverse if state.torque_on else 0
    safe_addstr(stdscr, row, 0, f"TORQUE : {torque_str}", torque_attr)
    row += 2

    # ---- 目標値 ----
    if state.is_position_like():
        safe_addstr(
            stdscr, row, 0,
            f"TARGET POSITION : {state.target_position:5d}  "
            f"(range {POSITION_MIN}-{POSITION_MAX}, step={state.position_step})",
        )
    else:
        safe_addstr(
            stdscr, row, 0,
            f"TARGET SPEED    : {state.target_speed:+5d}  "
            f"(range {SPEED_MIN}-{SPEED_MAX}, step={state.speed_step})",
        )
    row += 2

    # ---- 現在値 ----
    safe_addstr(stdscr, row, 0, "-- PRESENT (poll every %.1fs) --" % STATUS_POLL_INTERVAL)
    row += 1

    pp = state.present_position
    ps = state.present_speed
    pl = state.present_load
    mv = state.moving

    safe_addstr(
        stdscr, row, 0,
        f"Position={pp if pp is not None else '---':>5}  "
        f"Speed(raw)={ps if ps is not None else '---':>5}  "
        f"Load={pl if pl is not None else '---':>5}  "
        f"Moving={mv if mv is not None else '---'}",
    )
    row += 2

    # ---- エラー ----
    if state.last_error:
        safe_addstr(stdscr, row, 0, f"ERROR: {state.last_error}", bold)
    row += 2

    # ---- 操作ヘルプ ----
    help_lines = [
        "m        : モード切替 (切替後 torque は自動OFF)",
        "t        : torque ON/OFF トグル",
        "<- / ->  : 目標値を増減 (即送信)",
        "up / down: 増減幅(step)を変更",
        "r        : 目標値をサーボの現在値で再同期",
        "SPACE    : 現在の目標値を再送信",
        "q / ESC  : torque OFF -> 終了",
    ]

    for line in help_lines:
        safe_addstr(stdscr, row, 0, line)
        row += 1

    stdscr.refresh()


# =====================================================================
# キー入力処理
# =====================================================================

def handle_key(key, state):

    if key in (ord('m'), ord('M')):
        state.next_mode()

    elif key in (ord('t'), ord('T')):
        state.toggle_torque()

    elif key == curses.KEY_LEFT:
        state.adjust_target(-1)

    elif key == curses.KEY_RIGHT:
        state.adjust_target(+1)

    elif key == curses.KEY_UP:
        state.adjust_step(+1)

    elif key == curses.KEY_DOWN:
        state.adjust_step(-1)

    elif key in (ord('r'), ord('R')):
        state.sync_target_from_present()

    elif key == ord(' '):
        state.send_target()

    return key not in (ord('q'), ord('Q'), 27)  # 27 = ESC


# =====================================================================
# curses メインループ
# =====================================================================

def run(stdscr, servo):

    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.nodelay(True)
    stdscr.timeout(100)  # [ms] getch()のブロック上限

    state = AppState(servo)
    state.init_mode_from_servo()
    state.poll_status(force=True)

    running = True

    while running:

        key = stdscr.getch()

        if key != -1:
            running = handle_key(key, state)

        state.poll_status()
        draw(stdscr, state)

        time.sleep(LOOP_TICK)


# =====================================================================
# エントリポイント
# =====================================================================

def main():

    servo = STS3215(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=0.1,
    )

    servo.open()

    try:
        if not servo.ping(SERVO_ID):
            raise RuntimeError(f"Servo ID={SERVO_ID} not found (PING failed)")

        # sts3215.py の read()/write() は通信ログをprint()するため、
        # curses描画と衝突しないようログファイルへリダイレクトする。
        with open(DEBUG_LOG_PATH, "w") as log_file:
            with contextlib.redirect_stdout(log_file):
                curses.wrapper(run, servo)

    except Exception as e:
        print("ERROR:", type(e).__name__, e)

    finally:
        try:
            servo.torque(SERVO_ID, False)
        except Exception:
            pass

        servo.close()
        print("Serial closed")
        print(f"通信ログ: {DEBUG_LOG_PATH}")


if __name__ == "__main__":
    main()
