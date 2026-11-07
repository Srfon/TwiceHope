
#!/usr/bin/env python3

import os
import sys
import time
import socket
import threading
import io

import ST7789
from PIL import Image, ImageDraw, ImageFont

# ============================================================
# 設定
# ============================================================

SYSTEM_LOG = "/home/trin1/RobCup_V3/Display_Process/Log/system.log"
# 環境変数 DISPLAY_SOCKET_PATH で上書き可能 (systemd の PrivateTmp 対策で
# /run/display/display.sock などへ移すときに使う)。Logger.py 側も同じ値にすること
SOCKET_PATH = os.environ.get("DISPLAY_SOCKET_PATH", "/tmp/display.sock")
SOCKET_MODE = 0o666      # 別ユーザーのプロセスからも接続できるようにする
CLIENT_TIMEOUT = 5.0     # クライアントの無通信タイムアウト(秒)

DISPLAY_ROTATION = 90  # 左へ90度

DISPLAY_WIDTH = 240
DISPLAY_HEIGHT = 240
SPI_SPEED_HZ = 20_000_000

LOG_INTERVAL = 0.5
MAX_LOG_LINES = 8
MAX_IMAGE_SIZE = 10_000_000

BASE_IMAGE = "/home/trin1/RobCup_V3/Display_Process/Png/Base.png"

TAG_IMAGE = {
    "INFO": "/home/trin1/RobCup_V3/Display_Process/Png/info.png",
    "WARNING": "/home/trin1/RobCup_V3/Display_Process/Png/warning.png",
    "ERROR": "/home/trin1/RobCup_V3/Display_Process/Png/error.png",
    "DEBUG": "/home/trin1/RobCup_V3/Display_Process/Png/debug.png",
    "NOTICE": "/home/trin1/RobCup_V3/Display_Process/Png/notice.png",
}

# ------------------------------------------------------------
# Base.png (240x240) のログパネルレイアウト
# 1行目が最新、8行目が最古。ログパネル内に8行を配置する。
# テンプレート側の行番号(1..8)を使用し、タグ画像は描画しない。
# ------------------------------------------------------------

ROW_COUNT = 8
ROW_FIRST_Y = 140       # 1行目の文字中心Y。全体を下方向へ配置
ROW_PITCH = 12.0        # 8行を下部パネルいっぱいに配置
TEXT_X = 18              # 左余白を詰めて文字領域を拡大
TEXT_MAX_X = 230         # ログ文字の右端

# ログの先頭に付くレベル名だけを除去する。画面上にはタグを表示しない。
LOG_LEVELS = ("INFO", "DEBUG", "NOTICE", "WARNING", "WARN", "ERROR", "ERR")


# ============================================================
# LCD 初期化
# ============================================================

disp = ST7789.ST7789(
    port=0,
    cs=0,
    dc=25,
    rst=24,
    width=DISPLAY_WIDTH,
    height=DISPLAY_HEIGHT,
    rotation=0,
    spi_speed_hz=SPI_SPEED_HZ,
)

disp.begin()
disp._spi.mode = 3
disp.reset()
disp._init()

print("[LCD] ST7789 initialized")


# ============================================================
# 描画バッファ
# ============================================================

canvas = Image.new(
    "RGB",
    (DISPLAY_WIDTH, DISPLAY_HEIGHT),
    (0, 0, 0),
)

draw = ImageDraw.Draw(canvas)

# 描画処理の競合を防止
display_lock = threading.RLock()


# ============================================================
# フォント
# ============================================================

def load_font(size=12):
    """利用可能なフォントを読み込む。"""

    font_candidates = [
        # 日本語対応フォントがあれば優先
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKjp-Regular.otf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]

    for path in font_candidates:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                pass

    return ImageFont.load_default()


FONT = load_font(9)


# ============================================================
# LCD 制御
# ============================================================

def display_update():
    """左へ90度回転してLCDへ転送する。"""
    with display_lock:
        rotated = canvas.rotate(
            90,
            expand=False
        )
        disp.display(rotated)


def display_on():
    """LCD表示をONにする。"""

    with display_lock:
        disp.command(0x29)

    print("[LCD] Display ON")


def display_off():
    """LCD表示をOFFにする。"""

    with display_lock:
        disp.command(0x28)

    print("[LCD] Display OFF")


def display_clear(color=(0, 0, 0)):
    """描画バッファを指定色で塗りつぶしてLCDを更新する。"""

    global canvas, draw

    with display_lock:
        canvas = Image.new(
            "RGB",
            (DISPLAY_WIDTH, DISPLAY_HEIGHT),
            color,
        )

        draw = ImageDraw.Draw(canvas)

        rotated = canvas.rotate(DISPLAY_ROTATION, expand=False)
        disp.display(rotated)


# ============================================================
# 描画関数
# ============================================================

def draw_image(path, x=0, y=0):
    """
    画像を描画バッファに貼り付ける。

    path : 画像ファイル
    x, y : 左上の座標

    LCDへの転送は display_update() で実行する。
    """

    global draw

    try:
        with Image.open(path) as source:
            image = source.convert("RGBA")

        with display_lock:
            canvas.paste(
                image,
                (int(x), int(y)),
                image,
            )

            draw = ImageDraw.Draw(canvas)

        print(f"[DRAW IMAGE] {path} at ({x}, {y})")
        return True

    except Exception as e:
        print(f"[DRAW IMAGE ERROR] {path}: {e}")
        return False


def draw_text(text, x, y, color=(255, 255, 255)):
    """
    描画バッファに文字列を描画する。

    text  : 表示文字列
    x, y  : 左上の座標
    color : RGBカラー
    """

    global draw

    with display_lock:
        draw.text(
            (int(x), int(y)),
            str(text),
            font=FONT,
            fill=color,
        )

    print(f"[DRAW TEXT] '{text}' at ({x}, {y})")


def draw_image_data(image_data):
    """
    メモリ上の画像データをLCD全体に表示する。

    image_data : PNG/JPEGなどのバイナリデータ
    """

    global canvas, draw

    with Image.open(io.BytesIO(image_data)) as source:
        image = source.convert("RGB")

        if image.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
            image = image.resize(
                (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                Image.Resampling.LANCZOS,
            )

    with display_lock:
        canvas = image.copy()
        draw = ImageDraw.Draw(canvas)
        rotated = canvas.rotate(DISPLAY_ROTATION, expand=False)
        disp.display(rotated)


# ============================================================
# ログ表示管理
# ============================================================

class LogDisplayManager:

    def __init__(self):
        self.stop_log = threading.Event()
        self.shutdown_event = threading.Event()

        self.last_lines = []
        self.last_signature = None

    # --------------------------------------------------------
    # ログ読み込み
    # --------------------------------------------------------

    def read_log(self):
        """system.logの末尾6行を取得する。"""

        if not os.path.exists(SYSTEM_LOG):
            return []

        try:
            with open(
                SYSTEM_LOG,
                "r",
                encoding="utf-8",
                errors="replace",
            ) as f:
                lines = f.readlines()

            lines = [ln for ln in lines if ln.strip()]

            return lines[-MAX_LOG_LINES:]

        except OSError as e:
            print(f"[LOG ERROR] {e}")
            return []

    # --------------------------------------------------------
    # ログ解析
    # --------------------------------------------------------

    def parse_line(self, line):
        """ログレベルの接頭辞を除去し、画面用の本文を返す。"""
        text = line.strip()
        if ":" in text:
            prefix, remainder = text.split(":", 1)
            if prefix.strip().upper() in LOG_LEVELS:
                text = remainder.strip()
        return text

    # --------------------------------------------------------
    # 文字列を指定幅に収める
    # --------------------------------------------------------

    @staticmethod
    def fit_text(text, max_width):
        """max_width px に収まらない場合は末尾を '...' で省略する。"""

        if draw.textlength(text, font=FONT) <= max_width:
            return text

        ellipsis = "..."

        while text and draw.textlength(text + ellipsis, font=FONT) > max_width:
            text = text[:-1]

        return text + ellipsis

    # --------------------------------------------------------
    # ログ描画
    # --------------------------------------------------------

    def draw_log(self):
        """背景と最新8行を描画する。1行目が最新、8行目が最古。"""
        lines = self.read_log()
        self.last_lines = lines

        signature = tuple(lines)
        if signature == self.last_signature:
            return
        self.last_signature = signature

        global canvas, draw

        try:
            with Image.open(BASE_IMAGE) as source:
                background = source.convert("RGB")
            if background.size != (DISPLAY_WIDTH, DISPLAY_HEIGHT):
                background = background.resize(
                    (DISPLAY_WIDTH, DISPLAY_HEIGHT), Image.Resampling.LANCZOS
                )
        except Exception as e:
            print(f"[BACKGROUND ERROR] {e}")
            background = Image.new("RGB", (DISPLAY_WIDTH, DISPLAY_HEIGHT), (18, 31, 50))

        with display_lock:
            if self.stop_log.is_set():
                self.last_signature = None
                return

            canvas = background.copy()
            draw = ImageDraw.Draw(canvas)

            # 最新ログを最上段へ。ファイル末尾から逆順に並べる。
            newest_first = list(reversed(lines[-MAX_LOG_LINES:]))
            ref = draw.textbbox((0, 0), "Ag日", font=FONT)
            ref_center = (ref[1] + ref[3]) / 2

            for row, line in enumerate(newest_first):
                text = self.parse_line(line)
                y = ROW_FIRST_Y + row * ROW_PITCH

                # テンプレートにあるプレースホルダーの横棒を消してから文字を描く。
                # 行左端付近の背景色を使い、背景との色差を抑える。
                sample_y = max(0, min(DISPLAY_HEIGHT - 1, int(y + 4)))
                row_bg = background.getpixel((22, sample_y))
                # フォントサイズに合わせて行の背景を消去する。
                # 文字を大きくしても隣の行を消さないよう、行ピッチ内に収める。
                text_bbox = draw.textbbox((TEXT_X, 0), "Ag日", font=FONT)
                text_height = text_bbox[3] - text_bbox[1]
                clear_top = int(y - text_height / 2 - 1)
                clear_bottom = int(y + text_height / 2 + 1)
                draw.rectangle(
                    (16, clear_top, TEXT_MAX_X, clear_bottom),
                    fill=row_bg,
                )

                text = self.fit_text(text, TEXT_MAX_X - TEXT_X)
                draw.text(
                    (TEXT_X, round(y - ref_center)),
                    text,
                    font=FONT,
                    fill=(220, 231, 246),
                )

            rotated = canvas.rotate(DISPLAY_ROTATION, expand=False)
            disp.display(rotated)

        print(f"[LOG DISPLAY] {len(newest_first)} lines (newest first)")

    # --------------------------------------------------------
    # 常時ログ監視
    # --------------------------------------------------------

    def loop(self):
        """ログを定期監視してLCDへ表示する。"""

        while not self.shutdown_event.is_set():

            if not self.stop_log.is_set():
                try:
                    self.draw_log()

                except Exception as e:
                    print(f"[LOG DISPLAY ERROR] {e}")

            self.shutdown_event.wait(LOG_INTERVAL)

    # --------------------------------------------------------
    # 終了
    # --------------------------------------------------------

    def stop(self):
        """ログ監視を終了する。"""

        self.shutdown_event.set()


# ============================================================
# ソケット通信
# ============================================================

def recv_exact(conn, size):
    """指定サイズをすべて受信する。"""

    chunks = []
    remaining = size

    while remaining > 0:

        chunk = conn.recv(min(65536, remaining))

        if not chunk:
            raise ConnectionError(
                "画像データの受信中に接続が切断されました"
            )

        chunks.append(chunk)
        remaining -= len(chunk)

    return b"".join(chunks)


def socket_server(manager):
    """
    Unixドメインソケットで画像要求を受信する。

    通信形式:
        IMAGE:<拡張子>:<表示秒数>:<画像データサイズ>\n
        <画像バイナリデータ>
    """

    sock_dir = os.path.dirname(SOCKET_PATH) or "."

    if not os.path.isdir(sock_dir):
        os.makedirs(sock_dir, mode=0o755, exist_ok=True)

    # 古いソケットファイルが残っていれば削除
    try:
        os.remove(SOCKET_PATH)
    except FileNotFoundError:
        pass

    sock = socket.socket(
        socket.AF_UNIX,
        socket.SOCK_STREAM,
    )

    try:
        sock.bind(SOCKET_PATH)
    except OSError as e:
        print(
            f"[SOCKET BIND ERROR] {SOCKET_PATH}: {e} "
            f"(uid={os.getuid()})"
        )
        sock.close()
        raise

    # 実行ユーザー以外(例: 一般ユーザーのLogger.py)からも接続可能にする
    os.chmod(SOCKET_PATH, SOCKET_MODE)

    sock.listen(5)
    sock.settimeout(1.0)

    print(
        f"[DISPLAY MANAGER] Socket ready: {SOCKET_PATH} "
        f"(uid={os.getuid()}, mode={oct(SOCKET_MODE)})"
    )

    try:
        while not manager.shutdown_event.is_set():

            try:
                conn, _ = sock.accept()

            except socket.timeout:
                continue

            with conn:

                conn.settimeout(CLIENT_TIMEOUT)

                try:
                    # ヘッダーは改行まで受信
                    header_data = bytearray()

                    while not header_data.endswith(b"\n"):

                        chunk = conn.recv(1)

                        if not chunk:
                            raise ConnectionError(
                                "ヘッダー受信中に切断されました"
                            )

                        header_data.extend(chunk)

                        if len(header_data) > 256:
                            raise ValueError(
                                "ヘッダーが長すぎます"
                            )

                    header = header_data.decode(
                        "ascii"
                    ).strip()

                    parts = header.split(":")

                    if len(parts) != 4:
                        raise ValueError(
                            f"不正なヘッダー: {header}"
                        )

                    command = parts[0].upper()
                    ext = parts[1].lower().lstrip(".")
                    display_seconds = int(parts[2])
                    image_size = int(parts[3])

                    if command != "IMAGE":
                        raise ValueError(
                            f"不明なコマンド: {command}"
                        )

                    if ext not in ("png", "jpg", "jpeg"):
                        raise ValueError(
                            f"未対応の形式: {ext}"
                        )

                    if display_seconds < 0:
                        raise ValueError(
                            "表示時間は0以上にしてください"
                        )

                    if not (
                        1 <= image_size <= MAX_IMAGE_SIZE
                    ):
                        raise ValueError(
                            f"画像サイズが不正です: {image_size}"
                        )

                    # 画像バイナリを正確に受信
                    image_data = recv_exact(
                        conn,
                        image_size,
                    )

                    # 画像表示中はログ描画を停止
                    manager.stop_log.set()

                    try:
                        draw_image_data(image_data)

                        print(
                            f"[IMAGE DISPLAY] "
                            f"{ext}, "
                            f"{display_seconds}s, "
                            f"{image_size} bytes"
                        )

                        # 指定秒数表示
                        deadline = (
                            time.monotonic()
                            + display_seconds
                        )

                        while not manager.shutdown_event.is_set():

                            remaining = (
                                deadline - time.monotonic()
                            )

                            if remaining <= 0:
                                break

                            manager.shutdown_event.wait(
                                min(remaining, 0.1)
                            )

                    finally:
                        # 次回のログ描画を強制する
                        manager.last_signature = None
                        manager.stop_log.clear()

                except Exception as e:
                    print(f"[SOCKET CLIENT ERROR] {e}")

    except Exception as e:
        print(f"[SOCKET SERVER ERROR] {e}")

    finally:
        sock.close()

        if os.path.exists(SOCKET_PATH):
            os.remove(SOCKET_PATH)

        print("[DISPLAY MANAGER] Socket closed")


# ============================================================
# メイン
# ============================================================

def main():

    # systemd 配下でも print を即座に journal へ出す
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    manager = LogDisplayManager()

    # ログ監視スレッド
    log_thread = threading.Thread(
        target=manager.loop,
        daemon=True,
    )

    log_thread.start()

    try:
        display_on()
        socket_server(manager)

    except KeyboardInterrupt:
        print("\n[DISPLAY MANAGER] 終了要求を受け取りました")

    except Exception as e:
        print(f"[FATAL ERROR] {e}")

    finally:
        manager.stop()

        log_thread.join(timeout=2.0)

        try:
            display_off()

        except Exception as e:
            print(f"[LCD OFF ERROR] {e}")

        print("[DISPLAY MANAGER] 終了")


if __name__ == "__main__":
    main()