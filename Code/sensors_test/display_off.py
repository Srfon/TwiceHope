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

print("display off")

disp.command(0x28)

time.sleep(3)