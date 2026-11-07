"""
st7789_pygame_input.py
---------------------------------
WASD���̓A�j���[�V����
pygame��

�I��:
ESC
"""

import time
import math
import cv2
import numpy as np
import pygame

import ST7789
from PIL import Image

# ===========================================================
# pygame
# ===========================================================

pygame.init()

# �������s���E�B���h�E
pygame.display.set_mode((100, 100))

# ===========================================================
# ST7789
# ===========================================================

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

# ===========================================================
# CONFIG
# ===========================================================

W = 240
H = 240

x = W // 2
y = H // 2

speed = 3

t = 0.0

clock = pygame.time.Clock()

# ===========================================================
# LOOP
# ===========================================================

running = True

while running:

    # -------------------------------------------------------
    # EVENT
    # -------------------------------------------------------

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()

    # movement
    if keys[pygame.K_w]:
        y -= speed

    if keys[pygame.K_s]:
        y += speed

    if keys[pygame.K_a]:
        x -= speed

    if keys[pygame.K_d]:
        x += speed

    # exit
    if keys[pygame.K_ESCAPE]:
        running = False

    # limit
    x = max(10, min(W - 10, x))
    y = max(10, min(H - 10, y))

    # -------------------------------------------------------
    # DRAW
    # -------------------------------------------------------

    img = np.zeros((H, W, 3), dtype=np.uint8)

    # grid
    for gx in range(0, W, 20):
        cv2.line(img, (gx, 0), (gx, H), (25, 25, 25), 1)

    for gy in range(0, H, 20):
        cv2.line(img, (0, gy), (W, gy), (25, 25, 25), 1)

    # wave
    for i in range(0, W, 2):

        wy = int(
            H / 2 +
            math.sin(i * 0.06 + t * 3.0) * 15
        )

        cv2.circle(
            img,
            (i, wy),
            1,
            (255, 120, 120),
            -1
        )

    # player
    cv2.circle(
        img,
        (x, y),
        12,
        (0, 255, 255),
        -1,
        cv2.LINE_AA
    )

    # center
    cv2.circle(
        img,
        (W // 2, H // 2),
        4,
        (0, 0, 255),
        -1
    )

    # text
    cv2.putText(
        img,
        "WASD MOVE",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        img,
        "ESC EXIT",
        (10, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (180, 180, 180),
        1,
        cv2.LINE_AA
    )

    # -------------------------------------------------------
    # LCD OUTPUT
    # -------------------------------------------------------

    rgb = cv2.cvtColor(
        img,
        cv2.COLOR_BGR2RGB
    )

    pil_img = Image.fromarray(rgb)

    disp.display(pil_img)

    # time
    t += 0.05

    # fps
    clock.tick(60)

# ===========================================================
# EXIT
# ===========================================================

black = np.zeros((H, W, 3), dtype=np.uint8)

pil_black = Image.fromarray(
    cv2.cvtColor(black, cv2.COLOR_BGR2RGB)
)

disp.display(pil_black)

pygame.quit()

print("exit")