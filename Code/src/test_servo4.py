from servo.Servo_cli import STSBus
import servo.Servo_cli as servo
import time

MODE = 0          # 0:サーボ 1:速度 2:トルク 3:ステップ
REPEAT = 1        # 何回動かすか
AMOUNT = 1000     # 1回あたりの移動量/速度/トルク
SPEED = 200       # 速度 (mode 0/3 用)
INTERVAL = 2.0    # 1回ごとの待ち時間(秒)
IDS = None        # 動かすID。None なら自動スキャンの全ID。例: [1, 2, 3]
PORT = "/dev/ttyUSB0"
RELATIVE = True   # サーボモードを相対(積算)にするか
WRAP = False      # サーボモードで 4095 を超えたら 0 に巻き戻して周回し続けるか
                  # (連続回転は mode=3 ステップ推奨。これは応急処置)

bus = STSBus(PORT, mode=MODE, servo_relative=RELATIVE, servo_wrap=WRAP)


bus.torque_on(1)
bus.torque_on(2)

servo.servo_move(0,1,4000,500,ids=1,port=PORT,relative=RELATIVE,wrap=WRAP)

bus.torque_off(1)
bus.torque_off(2)
bus.close()