from gpiozero import Button
from PIL import Image, ImageDraw
import ST7789
import time

# =========================
# LCD 初期化
# =========================
disp = ST7789.ST7789(
    port=0,
    cs=0,
    dc=25,
    rst=24,
    width=240,
    height=240,
    rotation=0,
    spi_speed_hz=20_000_000
)

disp.begin()
disp._spi.mode = 3
disp.reset()
disp._init()

# =========================
# ボタン設定
# =========================
btn1 = Button(17)
btn2 = Button(18)
btn3 = Button(27)
btn4 = Button(22)

# =========================
# 描画関数
# =========================
def draw_text(msg):
    img = Image.new("RGB", (240, 240), (0, 0, 0))  # 黒背景
    draw = ImageDraw.Draw(img)
    draw.text((20, 110), msg, fill=(255, 255, 255))
    disp.display(img)

# =========================
# ボタンが押された時の処理
# =========================
def pressed(n):
    print(f"Button {n} pressed")
    draw_text(f"Button {n} pressed")

btn1.when_pressed = lambda: pressed(1)
btn2.when_pressed = lambda: pressed(2)
btn3.when_pressed = lambda: pressed(3)
btn4.when_pressed = lambda: pressed(4)

# =========================
# 初期画面
# =========================
draw_text("READY")

# =========================
# メインループ
# =========================
while True:
    time.sleep(0.1)
