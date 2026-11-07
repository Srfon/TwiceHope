import ST7789
from PIL import Image, ImageDraw
import time

# =========================
# LCD ������
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
# �`��
# =========================

img = Image.new("RGB", (240, 240), (0, 100, 255))
draw = ImageDraw.Draw(img)

draw.text((50, 110), "DISPLAY TEST", fill=(255,255,255))

disp.display(img)

print("display on")
time.sleep(3)

# =========================
# Display OFF
# =========================

print("display off")

disp.command(0x28)

time.sleep(3)

# =========================
# Display ON
# =========================

print("display on")

disp.command(0x29)

time.sleep(1)

# �ĕ`��
disp.display(img)

while True:
    time.sleep(1)