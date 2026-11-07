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

POWER_DEFAULT = 400

POWER_MIN = 100
POWER_MAX = 4096
POWER_STEP = 50


# ============================================================
# 90度旋回時間
# ============================================================

TURN_90_TIME = 0.75


# ============================================================
# モーター方向補正
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
        min(1023, magnitude)
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

        self.turn_end_time = None


    # --------------------------------------------------------
    # 初期化
    # --------------------------------------------------------

    def initialize(self):

        for motor_id in MOTOR_IDS:

            if not self.servo.ping(motor_id):

                raise RuntimeError(
                    f"Servo ID={motor_id} not found"
                )

        # Torque OFF
        for motor_id in MOTOR_IDS:

            self.servo.torque(
                motor_id,
                False
            )

        # Velocity mode
        for motor_id in MOTOR_IDS:

            self.servo.set_mode(
                motor_id,
                STS3215.MODE_VELOCITY
            )

        # Torque ON
        for motor_id in MOTOR_IDS:

            self.servo.torque(
                motor_id,
                True
            )

        self.stop()


    # --------------------------------------------------------
    # 1輪速度
    # --------------------------------------------------------

    def set_motor_speed(
        self,
        motor_id,
        speed
    ):

        speed *= MOTOR_DIRECTION[motor_id]

        raw = signed_to_raw(speed)

        self.servo.write_u16(
            motor_id,
            self.servo.ADDR_GOAL_SPEED,
            raw
        )


    # --------------------------------------------------------
    # 左右速度
    # --------------------------------------------------------

    def drive(
        self,
        left_speed,
        right_speed
    ):

        self.set_motor_speed(
            LEFT_FRONT,
            left_speed
        )

        self.set_motor_speed(
            LEFT_REAR,
            left_speed
        )

        self.set_motor_speed(
            RIGHT_FRONT,
            right_speed
        )

        self.set_motor_speed(
            RIGHT_REAR,
            right_speed
        )


    # ========================================================
    # 前進
    # ========================================================

    def forward(self):

        self.turn_end_time = None

        self.drive(
            self.power,
            self.power
        )

        self.motion = "FORWARD"


    # ========================================================
    # 後退
    # ========================================================

    def backward(self):

        self.turn_end_time = None

        self.drive(
            -self.power,
            -self.power
        )

        self.motion = "BACKWARD"


    # ========================================================
    # 停止
    # ========================================================

    def stop(self):

        self.turn_end_time = None

        self.drive(
            0,
            0
        )

        self.motion = "STOP"


    # ========================================================
    # 左90°
    #
    # ※実機で左右逆だったため前版から反転
    # ========================================================

    def left_90(self):

        # 左側を前進
        # 右側を後退

        self.drive(
            self.power,
            -self.power
        )

        self.motion = "LEFT 90"

        self.turn_end_time = (
            time.monotonic()
            + TURN_90_TIME
        )


    # ========================================================
    # 右90°
    #
    # ※実機で左右逆だったため前版から反転
    # ========================================================

    def right_90(self):

        # 左側を後退
        # 右側を前進

        self.drive(
            -self.power,
            self.power
        )

        self.motion = "RIGHT 90"

        self.turn_end_time = (
            time.monotonic()
            + TURN_90_TIME
        )


    # ========================================================
    # パワー変更
    # ========================================================

    def power_up(self):

        self.power += POWER_STEP

        self.power = min(
            POWER_MAX,
            self.power
        )


    def power_down(self):

        self.power -= POWER_STEP

        self.power = max(
            POWER_MIN,
            self.power
        )


    # ========================================================
    # 90度旋回終了判定
    # ========================================================

    def update(self):

        if self.turn_end_time is None:
            return

        if time.monotonic() >= self.turn_end_time:

            self.stop()


    # ========================================================
    # 終了処理
    # ========================================================

    def shutdown(self):

        try:
            self.stop()

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
# curses
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
# 画面
# ============================================================

def draw(
    stdscr,
    robot
):

    stdscr.erase()

    row = 0

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
        "=" * 45
    )

    row += 2


    # --------------------------------------------------------
    # 状態
    # --------------------------------------------------------

    safe_addstr(
        stdscr,
        row,
        0,
        f"STATE : {robot.motion}",
        curses.A_BOLD
    )

    row += 2


    # --------------------------------------------------------
    # パワー
    # --------------------------------------------------------

    safe_addstr(
        stdscr,
        row,
        0,
        f"POWER : {robot.power} / {POWER_MAX}",
        curses.A_BOLD
    )

    row += 1


    # 簡易パワーバー

    bar_length = 20

    filled = int(
        robot.power
        / POWER_MAX
        * bar_length
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


    safe_addstr(
        stdscr,
        row,
        0,
        f"90 deg time : {TURN_90_TIME:.3f} sec"
    )

    row += 2


    # --------------------------------------------------------
    # 操作説明
    # --------------------------------------------------------

    help_lines = [

        "W          : FORWARD",

        "S          : BACKWARD",

        "A          : LEFT 90 deg",

        "D          : RIGHT 90 deg",

        "SPACE      : STOP",

        "UP         : POWER +50",

        "DOWN       : POWER -50",

        "Q / ESC    : EXIT",

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
# キー処理
# ============================================================

def handle_key(
    key,
    robot
):

    # --------------------------------------------------------
    # W 前進
    # --------------------------------------------------------

    if key in (
        ord("w"),
        ord("W"),
    ):

        robot.forward()


    # --------------------------------------------------------
    # S 後退
    # --------------------------------------------------------

    elif key in (
        ord("s"),
        ord("S"),
    ):

        robot.backward()


    # --------------------------------------------------------
    # A 左90°
    # --------------------------------------------------------

    elif key in (
        ord("a"),
        ord("A"),
    ):

        robot.left_90()


    # --------------------------------------------------------
    # D 右90°
    # --------------------------------------------------------

    elif key in (
        ord("d"),
        ord("D"),
    ):

        robot.right_90()


    # --------------------------------------------------------
    # SPACE 停止
    # --------------------------------------------------------

    elif key == ord(" "):

        robot.stop()


    # --------------------------------------------------------
    # ↑ パワーアップ
    # --------------------------------------------------------

    elif key == curses.KEY_UP:

        robot.power_up()


    # --------------------------------------------------------
    # ↓ パワーダウン
    # --------------------------------------------------------

    elif key == curses.KEY_DOWN:

        robot.power_down()


    # --------------------------------------------------------
    # Q / ESC 終了
    # --------------------------------------------------------

    elif key in (
        ord("q"),
        ord("Q"),
        27
    ):

        return False

    return True


# ============================================================
# リアルタイムループ
# ============================================================

def run(
    stdscr,
    robot
):

    curses.curs_set(0)

    stdscr.keypad(True)

    stdscr.nodelay(True)

    stdscr.timeout(20)

    running = True

    while running:

        key = stdscr.getch()

        if key != -1:

            running = handle_key(
                key,
                robot
            )

        robot.update()

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


if __name__ == "__main__":

    main()