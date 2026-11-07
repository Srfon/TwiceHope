"""
robot.py  —  STS3032 2台を「短く書いて動かす」ための薄いラッパー

使い方(これだけ):
    from robot import Robot
    import time

    with Robot() as r:        # with を使うと終了時に自動で停止+クローズ
        r.forward()           # 前進
        time.sleep(2)
        r.right()             # 右回り
        time.sleep(1)
        r.stop()              # 止める

設定はこのファイル先頭の数行だけ、自分の機体に合わせて変える。
"""

import time
from Servo_cli3 import STSBus


# ================================================================
# 設定(ここだけ自分の機体に合わせる)
# ================================================================
PORT = "/dev/ttyUSB0"
DIRECTIONS = {1: -1, 2: 1}   # 前進の符号(確定済み)
LEFT_IDS = [1]               # 左輪のID
RIGHT_IDS = [2]              # 右輪のID
SPEED = 2000                 # 標準スピード


class Robot:
    """1語で動かせる走行ラッパー。"""

    def __init__(self, speed=SPEED):
        self.speed = speed
        self.bus = STSBus(
            PORT,
            mode=STSBus.MODE_SPEED,
            ids=LEFT_IDS + RIGHT_IDS,
            directions=DIRECTIONS,
            left_ids=LEFT_IDS,
            right_ids=RIGHT_IDS,
        )

    # --- 基本の動き(引数なしでOK。速さを変えたいときだけ渡す) ---
    def forward(self, speed=None):
        self.bus.forward(speed or self.speed)

    def back(self, speed=None):
        self.bus.backward(speed or self.speed)

    def left(self, speed=None):
        self.bus.spin_left(speed or self.speed)

    def right(self, speed=None):
        self.bus.spin_right(speed or self.speed)

    def drive(self, forward=0, turn=0):
        """前進量と旋回量を同時に指定(カーブ走行など)。"""
        self.bus.drive(forward=forward, turn=turn)

    def stop(self):
        self.bus.stop()

    def close(self):
        self.bus.stop()
        self.bus.close()

    # --- with 文に対応(終了時/エラー時に自動で停止+クローズ) ---
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # --- 制御ループ ---
    def loop(self, step, hz=20):
        """
        step() をくり返し呼ぶ制御ループ。
            ・step() が False を返すと終了
            ・Ctrl+C でも終了
            ・終了時は自動で停止+クローズ
        hz は1秒あたりの実行回数(既定20回 = 0.05秒ごと)。

        例:
            r = Robot()
            def step():
                # ここでセンサーを読んで動きを決める
                r.forward()
            r.loop(step)
        """
        dt = 1.0 / hz
        try:
            while True:
                if step() is False:
                    break
                time.sleep(dt)
        except KeyboardInterrupt:
            print("\n停止します")
        finally:
            self.close()


# ================================================================
# 実行サンプル(試したい例のコメントを外して実行)
# ================================================================
def demo_sequence():
    """順番に動かすだけのサンプル。"""
    with Robot() as r:
        r.forward(); time.sleep(2)
        r.back();    time.sleep(2)
        r.right();   time.sleep(1)
        r.left();    time.sleep(1)


def demo_control_loop():
    """
    制御ループのサンプル(センサーで動きを決める形)。
    read_target() を自分のセンサー(カメラ等)に置き換えればそのまま使える。
    """
    r = Robot()

    def read_target():
        # ★ ここを自分のセンサー読み取りに置き換える ★
        # 例: HuskyLens が見つけた物体の左右位置や、ライン位置 など。
        # ここではデモとして "center" を返しておく。
        return "center"

    def step():
        target = read_target()
        if target == "left":
            r.left()        # 左にズレてる -> 左へ向ける
        elif target == "right":
            r.right()       # 右にズレてる -> 右へ向ける
        elif target == "center":
            r.forward()     # 正面 -> 前進
        else:
            r.stop()        # 見失った -> 止まる
        # return False すればループを抜ける

    r.loop(step, hz=20)     # 0.05秒ごとに step を実行。Ctrl+Cで停止


if __name__ == "__main__":
    demo_sequence()
    # demo_control_loop()