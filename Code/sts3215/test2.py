#!/usr/bin/env python3

import contextlib
import curses
import time

from sts3215 import STS3215


# ============================================================
# 通信設定
# ============================================================

PORT = "/dev/ttyUSB0"
BAUDRATE = 1_000_000

DEBUG_LOG_PATH = "sts3215_4wd_debug.log"


# ============================================================
# モーターID
# ============================================================

LEFT_FRONT = 1
RIGHT_FRONT = 2
LEFT_REAR = 3
RIGHT_REAR = 4

MOTOR_IDS = [
    LEFT_FRONT,
    RIGHT_FRONT,
    LEFT_REAR,
    RIGHT_REAR,
]


# ============================================================
# パワー設定
# ============================================================

POWER_DEFAULT = 1000

POWER_MIN = 100
POWER_MAX = 4095

POWER_STEP = 100


# ============================================================
# キー長押し判定
#
# VS Code RemoteではキーRELEASEを取得できないため、
# キーリピートが止まったことで「離した」と判断する。
#
# 最初のキー入力からOSのキーリピートが始まるまでは
# INITIAL_HOLD_TIMEOUTだけ待つ。
#
# リピート開始後はREPEAT_HOLD_TIMEOUT以内に次の入力が
# 来なくなったら停止する。
# ============================================================

INITIAL_HOLD_TIMEOUT = 0.65
REPEAT_HOLD_TIMEOUT = 0.18


# ============================================================
# モーター方向補正
#
# 現在の実機で前進できている設定
# ============================================================

MOTOR_DIRECTION = {
    LEFT_FRONT: 1,
    RIGHT_FRONT: -1,
    LEFT_REAR: 1,
    RIGHT_REAR: -1,
}


# ============================================================
# 符号付き速度 -> STS3215 raw
# ============================================================

def signed_to_raw(value):

    value = int(value)

    magnitude = abs(value)

    magnitude = max(
        0,
        min(POWER_MAX, magnitude)
    )

    if value < 0:
        return magnitude | 0x8000

    return magnitude


# ============================================================
# 4輪駆動
# ============================================================

class FourWheelDrive:

    def __init__(self, servo):

        self.servo = servo

        self.power = POWER_DEFAULT

        self.motion = "STOP"

        # 現在押されていると判断しているキー
        self.active_key = None

        # 最後にキー入力を受信した時間
        self.last_move_key_time = 0.0

        # 最初に押した時間
        self.first_move_key_time = 0.0

        # キーリピート開始済みか
        self.repeat_started = False

        # 同じキーを何回受信したか
        self.key_receive_count = 0


    # ========================================================
    # 初期化
    # ========================================================

    def initialize(self):

        # ----------------------------------------------------
        # 全サーボPING
        # ----------------------------------------------------

        for motor_id in MOTOR_IDS:

            if not self.servo.ping(motor_id):

                raise RuntimeError(
                    f"Servo ID={motor_id} not found"
                )


        # ----------------------------------------------------
        # Torque OFF
        # ----------------------------------------------------

        for motor_id in MOTOR_IDS:

            self.servo.torque(
                motor_id,
                False
            )


        # ----------------------------------------------------
        # VELOCITY MODE
        # ----------------------------------------------------

        for motor_id in MOTOR_IDS:

            self.servo.set_mode(
                motor_id,
                STS3215.MODE_VELOCITY
            )


        # ----------------------------------------------------
        # Torque ON
        # ----------------------------------------------------

        for motor_id in MOTOR_IDS:

            self.servo.torque(
                motor_id,
                True
            )


        # ----------------------------------------------------
        # 初期停止
        # ----------------------------------------------------

        self.stop()


    # ========================================================
    # 1モーター速度
    # ========================================================

    def set_motor_speed(
        self,
        motor_id,
        speed
    ):

        corrected_speed = (
            speed
            * MOTOR_DIRECTION[motor_id]
        )

        raw = signed_to_raw(
            corrected_speed
        )

        self.servo.write_u16(
            motor_id,
            self.servo.ADDR_GOAL_SPEED,
            raw
        )


    # ========================================================
    # 左右駆動
    # ========================================================

    def drive(
        self,
        left_speed,
        right_speed
    ):

        # 左前
        self.set_motor_speed(
            LEFT_FRONT,
            left_speed
        )

        # 左後
        self.set_motor_speed(
            LEFT_REAR,
            left_speed
        )

        # 右前
        self.set_motor_speed(
            RIGHT_FRONT,
            right_speed
        )

        # 右後
        self.set_motor_speed(
            RIGHT_REAR,
            right_speed
        )


    # ========================================================
    # 前進
    # ========================================================

    def forward(self):

        self.drive(
            self.power,
            self.power
        )

        self.motion = "FORWARD"


    # ========================================================
    # 後退
    # ========================================================

    def backward(self):

        self.drive(
            -self.power,
            -self.power
        )

        self.motion = "BACKWARD"


    # ========================================================
    # 左旋回
    #
    # 実機で左右逆だったため修正済み
    # ========================================================

    def left(self):

        self.drive(
            self.power,
            -self.power
        )

        self.motion = "LEFT"


    # ========================================================
    # 右旋回
    # ========================================================

    def right(self):

        self.drive(
            -self.power,
            self.power
        )

        self.motion = "RIGHT"


    # ========================================================
    # 停止
    # ========================================================

    def stop(self):

        self.drive(
            0,
            0
        )

        self.motion = "STOP"


    # ========================================================
    # 移動キー入力
    # ========================================================

    def move_key(self, key):

        now = time.monotonic()


        # ----------------------------------------------------
        # 新しいキーに変更された
        # ----------------------------------------------------

        if key != self.active_key:

            self.active_key = key

            self.first_move_key_time = now

            self.last_move_key_time = now

            self.repeat_started = False

            self.key_receive_count = 1


        # ----------------------------------------------------
        # 同じキーが再入力された
        #
        # キーリピート開始と判断
        # ----------------------------------------------------

        else:

            self.key_receive_count += 1

            self.last_move_key_time = now

            if self.key_receive_count >= 2:

                self.repeat_started = True


        # ----------------------------------------------------
        # 動作実行
        # ----------------------------------------------------

        self.apply_active_key()


    # ========================================================
    # 現在のキーに応じて走行
    # ========================================================

    def apply_active_key(self):

        if self.active_key == "W":

            self.forward()


        elif self.active_key == "S":

            self.backward()


        elif self.active_key == "A":

            self.left()


        elif self.active_key == "D":

            self.right()


    # ========================================================
    # キーが離されたか監視
    # ========================================================

    def update_key_hold(self):

        if self.active_key is None:
            return


        now = time.monotonic()


        # ----------------------------------------------------
        # キーリピート開始前
        # ----------------------------------------------------

        if not self.repeat_started:

            elapsed = (
                now
                - self.first_move_key_time
            )

            timeout = INITIAL_HOLD_TIMEOUT


        # ----------------------------------------------------
        # キーリピート開始後
        # ----------------------------------------------------

        else:

            elapsed = (
                now
                - self.last_move_key_time
            )

            timeout = REPEAT_HOLD_TIMEOUT


        # ----------------------------------------------------
        # 入力が来なくなった
        # = キーを離したと判断
        # ----------------------------------------------------

        if elapsed > timeout:

            self.clear_move_key()

            self.stop()


    # ========================================================
    # 移動キー解除
    # ========================================================

    def clear_move_key(self):

        self.active_key = None

        self.repeat_started = False

        self.key_receive_count = 0

        self.last_move_key_time = 0.0

        self.first_move_key_time = 0.0


    # ========================================================
    # POWER UP
    # ========================================================

    def power_up(self):

        self.power += POWER_STEP

        self.power = min(
            POWER_MAX,
            self.power
        )


        # 走行中なら即座に反映
        if self.active_key is not None:

            self.apply_active_key()


    # ========================================================
    # POWER DOWN
    # ========================================================

    def power_down(self):

        self.power -= POWER_STEP

        self.power = max(
            POWER_MIN,
            self.power
        )


        # 走行中なら即座に反映
        if self.active_key is not None:

            self.apply_active_key()


    # ========================================================
    # 強制停止
    # ========================================================

    def emergency_stop(self):

        self.clear_move_key()

        self.stop()


    # ========================================================
    # 終了
    # ========================================================

    def shutdown(self):

        try:

            self.emergency_stop()

        except Exception:

            pass


        for motor_id in MOTOR_IDS:

            try:

                self.servo.torque(
                    motor_id,
                    False
                )

            except Exception:

                pass


# ============================================================
# curses安全描画
# ============================================================

def safe_addstr(
    stdscr,
    y,
    x,
    text,
    attr=0
):

    try:

        stdscr.addstr(
            y,
            x,
            text,
            attr
        )

    except curses.error:

        pass


# ============================================================
# 画面描画
# ============================================================

def draw(
    stdscr,
    robot
):

    stdscr.erase()

    row = 0


    # ========================================================
    # タイトル
    # ========================================================

    safe_addstr(
        stdscr,
        row,
        0,
        "STS3215 4WD REALTIME CONTROL",
        curses.A_BOLD
    )

    row += 1


    safe_addstr(
        stdscr,
        row,
        0,
        "=" * 50
    )

    row += 2


    # ========================================================
    # 状態
    # ========================================================

    safe_addstr(
        stdscr,
        row,
        0,
        f"STATE : {robot.motion}",
        curses.A_BOLD
    )

    row += 1


    # ========================================================
    # 現在の移動キー
    # ========================================================

    active = (
        robot.active_key
        if robot.active_key is not None
        else "---"
    )

    safe_addstr(
        stdscr,
        row,
        0,
        f"KEY   : {active}"
    )

    row += 2


    # ========================================================
    # POWER
    # ========================================================

    safe_addstr(
        stdscr,
        row,
        0,
        f"POWER : {robot.power} / {POWER_MAX}",
        curses.A_BOLD
    )

    row += 1


    # ========================================================
    # POWERバー
    # ========================================================

    bar_length = 30

    filled = int(
        robot.power
        / POWER_MAX
        * bar_length
    )

    filled = max(
        0,
        min(
            bar_length,
            filled
        )
    )


    bar = (
        "["
        + "#" * filled
        + "-" * (bar_length - filled)
        + "]"
    )


    safe_addstr(
        stdscr,
        row,
        0,
        bar
    )

    row += 2


    # ========================================================
    # キーリピート状態
    # ========================================================

    repeat_text = (
        "YES"
        if robot.repeat_started
        else "NO"
    )

    safe_addstr(
        stdscr,
        row,
        0,
        f"KEY REPEAT : {repeat_text}"
    )

    row += 2


    # ========================================================
    # 操作説明
    # ========================================================

    help_lines = [

        "Hold W       : FORWARD",

        "Hold S       : BACKWARD",

        "Hold A       : TURN LEFT",

        "Hold D       : TURN RIGHT",

        "",

        f"UP           : POWER +{POWER_STEP}",

        f"DOWN         : POWER -{POWER_STEP}",

        "",

        "SPACE        : STOP",

        "Q / ESC      : EXIT",

    ]


    for line in help_lines:

        safe_addstr(
            stdscr,
            row,
            0,
            line
        )

        row += 1


    stdscr.refresh()


# ============================================================
# キー入力
# ============================================================

def handle_key(
    key,
    robot
):

    # ========================================================
    # W 前進
    # ========================================================

    if key in (
        ord("w"),
        ord("W"),
    ):

        robot.move_key("W")


    # ========================================================
    # S 後退
    # ========================================================

    elif key in (
        ord("s"),
        ord("S"),
    ):

        robot.move_key("S")


    # ========================================================
    # A 左
    # ========================================================

    elif key in (
        ord("a"),
        ord("A"),
    ):

        robot.move_key("A")


    # ========================================================
    # D 右
    # ========================================================

    elif key in (
        ord("d"),
        ord("D"),
    ):

        robot.move_key("D")


    # ========================================================
    # ↑ POWER UP
    # ========================================================

    elif key == curses.KEY_UP:

        robot.power_up()


    # ========================================================
    # ↓ POWER DOWN
    # ========================================================

    elif key == curses.KEY_DOWN:

        robot.power_down()


    # ========================================================
    # SPACE 強制停止
    # ========================================================

    elif key == ord(" "):

        robot.emergency_stop()


    # ========================================================
    # Q / ESC 終了
    # ========================================================

    elif key in (
        ord("q"),
        ord("Q"),
        27,
    ):

        robot.emergency_stop()

        return False


    return True


# ============================================================
# リアルタイムメインループ
# ============================================================

def run(
    stdscr,
    robot
):

    curses.curs_set(0)

    stdscr.keypad(True)

    stdscr.nodelay(True)

    # 20ms
    stdscr.timeout(20)


    running = True


    while running:

        # ----------------------------------------------------
        # キー取得
        # ----------------------------------------------------

        key = stdscr.getch()


        if key != -1:

            running = handle_key(
                key,
                robot
            )


        # ----------------------------------------------------
        # キー長押し監視
        # ----------------------------------------------------

        robot.update_key_hold()


        # ----------------------------------------------------
        # 表示
        # ----------------------------------------------------

        draw(
            stdscr,
            robot
        )


        time.sleep(0.01)


# ============================================================
# main
# ============================================================

def main():

    servo = STS3215(
        port=PORT,
        baudrate=BAUDRATE,
        timeout=0.1
    )


    servo.open()


    robot = FourWheelDrive(
        servo
    )


    try:

        # ----------------------------------------------------
        # sts3215.py の通信ログをファイルへ
        # ----------------------------------------------------

        with open(
            DEBUG_LOG_PATH,
            "w"
        ) as log_file:


            with contextlib.redirect_stdout(
                log_file
            ):


                robot.initialize()


                curses.wrapper(
                    run,
                    robot
                )


    except Exception as e:

        print(
            "ERROR:",
            type(e).__name__,
            e
        )


    finally:

        try:

            robot.shutdown()

        except Exception:

            pass


        servo.close()


        print("Serial closed")

        print(
            f"通信ログ: {DEBUG_LOG_PATH}"
        )


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":

    main()