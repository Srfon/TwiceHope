
#!/usr/bin/env python3

import os
import sys
import time
import socket
import threading
import io
import math
import fcntl
import shutil
import subprocess
from pathlib import Path

import ST7789
from PIL import Image, ImageDraw, ImageFont

# ============================================================
# 設定
# ============================================================

SYSTEM_LOG = "/home/trin1/RobCup_V3/Display_Process/Log/system.log"
# 環境変数 DISPLAY_SOCKET_PATH で上書き可能 (systemd の PrivateTmp 対策で
# /run/display/display.sock などへ移すときに使う)。Logger.py 側も同じ値にすること
SOCKET_PATH = os.environ.get("DISPLAY_SOCKET_PATH", "/tmp/display.sock")

# WebプレビューはソケットではなくRAM上のPNGファイルを監視する。
# systemd PrivateTmp の影響を受けない /dev/shm を使用。
LCD_MIRROR_PATH = os.environ.get("LCD_MIRROR_PATH", "/dev/shm/robocup_lcd_web.png")
LCD_MIRROR_MAX_AGE_SECONDS = 3.0   # 更新が止まった場合はログ画面へ戻す
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
# robot_mapping5.py の起動/終了に合わせ、マップPNGより2秒間優先表示する。
PANEL_PNG_DIR = Path(os.environ.get("ROBOT_PANEL_PNG_DIR", "/home/trin1/RobCup_V3/Display_Process/Png"))
ROBOT_START_IMAGE = PANEL_PNG_DIR / "rb_start.png"
ROBOT_END_IMAGE = PANEL_PNG_DIR / "rb_end.png"
ROBOT_RETRY_IMAGE = PANEL_PNG_DIR / "rb_retry.png"
RETRY_SIGNAL_PATH = Path(os.environ.get(
    "ROBOT_RETRY_SIGNAL", "/dev/shm/robocup_lcd_retry.signal"))
ROBOT_LOGO_HOLD_SECONDS = 2.0

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
# GPIO22: robot_mapping5.py が停止中だけ2秒長押しで起動
# ============================================================
ROBOT_SCRIPT = Path(os.environ.get(
    "ROBOT_SCRIPT", "/home/trin1/RobCup_V3/sts3215/robot_mapping5_2.py"))
ROBOT_PYTHON = Path(os.environ.get(
    "ROBOT_PYTHON", "/home/trin1/RobCup_V3/.venv/bin/python"))
ROBOT_ROOT = ROBOT_SCRIPT.parent.parent
ROBOT_LOCK_FILE = Path("/tmp/robocup_robot_mapping5.lock")
BUTTON1_PIN = 22
# GPIO22はdisplay_managerのみが確保。起動中のロボットへ押下をUNIX datagramで転送。
ROBOT_BUTTON_SOCKET = Path(os.environ.get("ROBOT_BUTTON_SOCKET", "/dev/shm/robocup_gpio22_press.sock"))
BUTTON_HOLD_SECONDS = 2.0
BUTTON_POLL_SECONDS = 0.025
# 起動要求から実際にロックを取得するまで、ボタンの再確保を保留する
ROBOT_BOOT_GRACE_SECONDS = 12.0


def _robot_is_running() -> bool:
    """既存のロボットを検知。systemd PrivateTmp が有効でも /proc を利用。"""
    # 同じ /tmp が見える場合は flock を最優先。ロックファイルは新規作成しない。
    try:
        fd = os.open(ROBOT_LOCK_FILE, os.O_RDONLY | os.O_CLOEXEC)
    except FileNotFoundError:
        pass
    except PermissionError:
        # 読めないロックは安全側へ倒す
        return True
    else:
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return True
            else:
                fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)
    # systemd PrivateTmp や debugpy でも正しい判定ができるよう /proc を確認。
    try:
        pids = os.listdir("/proc")
    except OSError:
        # /proc が取得できない時は安全側に倒す（自動起動しない）
        return True
    for name in pids:
        if not name.isdecimal() or int(name) == os.getpid():
            continue
        try:
            raw = Path(f"/proc/{name}/cmdline").read_bytes()
            args = [os.fsdecode(b) for b in raw.split(b"\0") if b]
        except (OSError, ValueError):
            continue
        if not args:
            continue
        try:
            # Zombie no longer executes code and must not retain the running state.
            if Path(f"/proc/{name}/stat").read_text().split(") ", 1)[1].split()[0] == "Z":
                continue
        except (OSError, IndexError):
            continue
        binary = os.path.basename(args[0]).lower()
        if not ("python" in binary or "debugpy" in binary):
            continue
        if any(os.path.basename(arg) == ROBOT_SCRIPT.name for arg in args[1:]):
            return True
    return False


class IdleRobotButtonMonitor:
    """GPIO22の唯一の所有者。停止中は長押し起動、起動中は押下イベント送信。"""

    def __init__(self, shutdown_event: threading.Event, on_state_change=None):
        self.shutdown_event = shutdown_event
        self.on_state_change = on_state_change
        self.launch_requested = threading.Event()
        self.button = None
        self._press_started_at = None
        self._hold_triggered = False
        self._pressed_prev = False
        self._lock = threading.RLock()
        self._startup_hold_until = 0.0
        self._log_busy = False
        self._thread = threading.Thread(target=self._loop,
                                        name="idle-robot-button1", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._thread.join(timeout=2.0)
        self._release_button()

    def _forward_press(self) -> None:
        """ロボット側のGPIO22イベント受信ソケットへ1押下を配送。"""
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
                client.settimeout(0.10)
                client.sendto(b"PRESS", str(ROBOT_BUTTON_SOCKET))
        except (OSError, TimeoutError) as exc:
            # ロボットの起動直後は受信ソケットがまだ用意されない場合がある。
            print(f"[GPIO22] ロボットへの押下転送失敗（受信待機中）: {exc}")

    def _poll_button(self, active: bool, allow_launch: bool) -> None:
        """同じGPIO入力から稼働中は押下、停止中は2秒長押しを判定する。"""
        with self._lock:
            button = self.button
            if button is None:
                return
            try:
                pressed = bool(button.is_pressed)
            except Exception as exc:
                print(f"[GPIO22] ボタン読み取り失敗: {exc}")
                self._release_button()
                return
            rising = pressed and not self._pressed_prev
            self._pressed_prev = pressed
            if active:
                # display_managerがGPIOを保持したままイベントだけ送る。
                self._press_started_at = None
                self._hold_triggered = False
                if rising:
                    self._forward_press()
                return
            if not pressed:
                self._press_started_at = None
                self._hold_triggered = False
                return
            if not allow_launch or self._hold_triggered:
                return
            now = time.monotonic()
            if self._press_started_at is None:
                self._press_started_at = now
            elif now - self._press_started_at >= BUTTON_HOLD_SECONDS:
                self._hold_triggered = True
                print("[GPIO22] Button1を2秒間長押し: ロボット起動要求")
                self.launch_requested.set()

    def _release_button(self) -> None:
        with self._lock:
            button, self.button = self.button, None
            self._press_started_at = None
            self._pressed_prev = False
        if button is not None:
            try:
                button.close()
            except Exception as exc:
                print(f"[GPIO22] ピン解放に失敗: {exc}")

    def _acquire_button(self) -> None:
        try:
            from gpiozero import Button
            button = Button(BUTTON1_PIN, pull_up=True, bounce_time=0.05)
        except Exception as exc:
            if not self._log_busy:
                print(f"[GPIO22] ボタン監視の初期化を再試行します: {exc}")
                self._log_busy = True
            return
        with self._lock:
            self.button = button
        self._log_busy = False
        print("[GPIO22] GPIO22監視開始。停止中はButton1の2秒長押し、稼働中は押下転送")

    def _launch_robot(self) -> bool:
        """systemd環境では別のユニットとして起動し、LCD再起動と分離する。"""
        if not ROBOT_SCRIPT.is_file() or not ROBOT_PYTHON.is_file():
            print(f"[GPIO22 ERROR] 起動ファイルがありません: {ROBOT_SCRIPT} / {ROBOT_PYTHON}")
            return False
        command = [str(ROBOT_PYTHON), "-u", str(ROBOT_SCRIPT), "--lcd-mode", "map"]
        try:
            if os.geteuid() == 0 and shutil.which("systemd-run"):
                unit = f"robocup-robot-button-{int(time.time())}-{os.getpid()}"
                result = subprocess.run([
                    "systemd-run", "--quiet", f"--unit={unit}", "--collect",
                    "--property=Type=exec", "--property=Restart=no",
                    "--property=StandardInput=null", "--uid=trin1",
                    f"--working-directory={ROBOT_ROOT}", *command,
                ], capture_output=True, text=True, timeout=10)
                if result.returncode != 0:
                    print(f"[GPIO22 ERROR] systemd-runが起動に失敗: {result.stderr.strip()}")
                    return False
                print(f"[GPIO22] robot_mapping5.pyを起動: systemdユニット {unit}.service")
                return True
            # systemdのroot権限がない場合も稼働させる。サービス再起動時に
            # 同一cgroupの子プロセスが終了することがあるため注意。
            logfile = ROBOT_ROOT / "robot_button_start.log"
            with open(logfile, "ab", buffering=0) as output:
                process = subprocess.Popen(
                    command, cwd=str(ROBOT_ROOT), stdin=subprocess.DEVNULL,
                    stdout=output, stderr=subprocess.STDOUT,
                    start_new_session=True, close_fds=True,
                )
            print(f"[GPIO22] robot_mapping5.pyを起動 PID={process.pid}, log={logfile}")
            return True
        except Exception as exc:
            print(f"[GPIO22 ERROR] ロボット起動失敗: {exc}")
            return False

    def _loop(self) -> None:
        was_running = False
        observed_running = None  # display_manager自身の再起動ではロゴを出さない
        print("[GPIO22] 単一所有方式: 管理側がGPIO22を常時監視 / ロボットへ押下イベント送信")
        while not self.shutdown_event.is_set():
            active = _robot_is_running()
            if observed_running is None:
                observed_running = active
            elif active != observed_running:
                observed_running = active
                if self.on_state_change is not None:
                    try:
                        self.on_state_change("start" if active else "end")
                    except Exception as exc:
                        print(f"[ROBOT LOGO] 切替失敗: {exc}")
            if self.button is None:
                self._acquire_button()
            if active:
                if not was_running:
                    print("[GPIO22] ロボット実行中: GPIO22の押下をロボットへ転送")
                was_running = True
                self.launch_requested.clear()
                self._startup_hold_until = 0.0
            else:
                if was_running:
                    print("[GPIO22] ロボット終了: Button1の2秒長押し待機に復帰")
                was_running = False
            grace_active = time.monotonic() < self._startup_hold_until
            self._poll_button(active=active, allow_launch=not grace_active)
            if not active and not grace_active and self.launch_requested.is_set():
                self.launch_requested.clear()
                if not _robot_is_running() and self._launch_robot():
                    self._startup_hold_until = time.monotonic() + ROBOT_BOOT_GRACE_SECONDS
            self.shutdown_event.wait(BUTTON_POLL_SECONDS)
        self._release_button()



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
        self.frame_expires_at = 0.0  # None: FRAME 0で無期限保持
        self._mirror_last_signature = None
        self._mirror_bad_signature = None
        self._overlay_until = 0.0
        self._overlay_name = None
        # 古いretry通知をサービス再起動で再表示しない。
        try:
            self._retry_signature = RETRY_SIGNAL_PATH.stat().st_mtime_ns
        except OSError:
            self._retry_signature = None

    def show_transition(self, transition: str) -> bool:
        """ロボット起動/終了時、マップPNGより優先してロゴを2秒表示する。"""
        path = {"start": ROBOT_START_IMAGE, "end": ROBOT_END_IMAGE, "retry": ROBOT_RETRY_IMAGE}.get(transition)
        if path is None:
            raise ValueError(f"不明な遷移名: {transition}")
        try:
            payload = path.read_bytes()
            if not 1 <= len(payload) <= MAX_IMAGE_SIZE:
                raise ValueError(f"ロゴ画像サイズが不正: {len(payload)} bytes")
            if not payload.startswith(b"\x89PNG\r\n\x1a\n"):
                raise ValueError("PNG画像ではありません")
            with display_lock:
                # 描画完了時を起点に2秒保持する。
                draw_image_data(payload)
                self.stop_log.set()
                self._overlay_until = time.monotonic() + ROBOT_LOGO_HOLD_SECONDS
                self._overlay_name = transition
                self.frame_expires_at = None
                self._mirror_last_signature = None  # ロゴ終了後は同じマップPNGでも再描画
                self.last_signature = None
            print(f"[ROBOT LOGO] {transition.upper()}: {path} ({ROBOT_LOGO_HOLD_SECONDS:g}秒)")
            return True
        except (OSError, ValueError) as exc:
            print(f"[ROBOT LOGO] {transition.upper()}表示失敗: {exc}")
            return False

    def show_frame(self, image_data: bytes, hold_seconds: float,
                   persistent_zero: bool = True) -> None:
        """ログの上書きを抑制し、LCDへ最新の受信画像を表示する。"""
        with display_lock:
            if time.monotonic() < self._overlay_until:
                return  # 起動・終了ロゴを通常のFRAME/IMAGEで上書きしない
            self.stop_log.set()
            # 先に画像を検証してから既存画面を入れ替える。
            try:
                draw_image_data(image_data)
            except Exception:
                self.stop_log.clear()
                self.last_signature = None
                raise
            self.frame_expires_at = (
                None if hold_seconds == 0 and persistent_zero else time.monotonic() + hold_seconds
            )

    def expire_frame_if_needed(self) -> None:
        """ロゴ表示2秒終了後、マップPNGまたはログ画面へ戻す。"""
        with display_lock:
            if self._overlay_until:
                if time.monotonic() < self._overlay_until:
                    return
                print(f"[ROBOT LOGO] {self._overlay_name.upper()}表示終了")
                self._overlay_until = 0.0
                self._overlay_name = None
                self.frame_expires_at = 0.0
                self._mirror_last_signature = None
                self.last_signature = None
                self.stop_log.clear()
            expires = self.frame_expires_at
            if (self.stop_log.is_set() and expires is not None
                    and time.monotonic() >= expires):
                self.last_signature = None
                self.stop_log.clear()

    def poll_mirror_file(self) -> None:
        """完成済みPNGが新しくなった場合だけLCDへ転送する。"""
        # LCD描画は1つのディスプレイマネージャーで優先順位を管理する。
        if time.monotonic() < self._overlay_until:
            return
        try:
            st = os.stat(LCD_MIRROR_PATH)
        except FileNotFoundError:
            return  # ミラー未起動なら従来のログ表示
        except OSError as exc:
            print(f"[LCD MIRROR] stat失敗: {exc}")
            return
        # 終了済みプロセスの古い画像を表示し続けない
        if time.time() - st.st_mtime > LCD_MIRROR_MAX_AGE_SECONDS:
            return
        signature = (st.st_mtime_ns, st.st_size)
        if signature == self._mirror_last_signature or signature == self._mirror_bad_signature:
            return
        if not 1 <= st.st_size <= MAX_IMAGE_SIZE:
            self._mirror_bad_signature = signature
            print("[LCD MIRROR] PNGのサイズが不正です")
            return
        try:
            with open(LCD_MIRROR_PATH, "rb") as f:
                image_data = f.read()
            if not image_data.startswith(b"\x89PNG\r\n\x1a\n"):
                raise ValueError("PNGヘッダーではありません")
            # 既存のLCD描画経路を利用。FRAME/IMAGEのソケット通信は不要。
            self.show_frame(image_data, LCD_MIRROR_MAX_AGE_SECONDS,
                            persistent_zero=False)
        except (OSError, ValueError) as exc:
            self._mirror_bad_signature = signature
            print(f"[LCD MIRROR] 表示失敗: {exc}")
            return
        self._mirror_bad_signature = None
        self._mirror_last_signature = signature

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

    def poll_retry_signal(self) -> None:
        """実行中GPIO27緊急停止時のリトライ表示を最優先させる。"""
        try:
            signature = RETRY_SIGNAL_PATH.stat().st_mtime_ns
        except FileNotFoundError:
            return
        except OSError as exc:
            print(f"[ROBOT LOGO] retry通知の読み取り失敗: {exc}")
            return
        if signature != self._retry_signature:
            self._retry_signature = signature
            self.show_transition("retry")

    def loop(self):
        """ログを定期監視してLCDへ表示する。"""

        while not self.shutdown_event.is_set():

            self.poll_retry_signal()
            self.expire_frame_if_needed()
            self.poll_mirror_file()
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
    """IMAGE / FRAME画像をUnixドメインソケットで受信する。

    IMAGE:<形式>:<秒数>:<バイト数>\n + バイナリ ... 従来互換
    FRAME:<形式>:<秒数>:<バイト数>\n + バイナリ ... 更新優先（待たない）
    FRAME:...:0:... は次のフレームまで維持する。
    """
    sock_dir = os.path.dirname(SOCKET_PATH) or "."
    os.makedirs(sock_dir, mode=0o755, exist_ok=True)
    # 停止後に残ったソケットのみ削除。別プロセスが動いていれば上書きしない。
    if os.path.exists(SOCKET_PATH):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as probe:
            probe.settimeout(0.2)
            try:
                probe.connect(SOCKET_PATH)
            except (ConnectionRefusedError, FileNotFoundError):
                os.unlink(SOCKET_PATH)
            else:
                raise RuntimeError(f"既にディスプレイマネージャーが起動しています: {SOCKET_PATH}")

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.bind(SOCKET_PATH)
        os.chmod(SOCKET_PATH, SOCKET_MODE)
        sock.listen(5)
        sock.settimeout(1.0)
        print(f"[DISPLAY MANAGER] Socket ready: {SOCKET_PATH} "
              f"(uid={os.getuid()}, mode={oct(SOCKET_MODE)})")
        while not manager.shutdown_event.is_set():
            try:
                conn, _ = sock.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(CLIENT_TIMEOUT)
                try:
                    header_data = bytearray()
                    while not header_data.endswith(b"\n"):
                        part = conn.recv(1)
                        if not part:
                            raise ConnectionError("ヘッダー受信中に切断されました")
                        header_data.extend(part)
                        if len(header_data) > 256:
                            raise ValueError("ヘッダーが長すぎます")
                    header = header_data.decode("ascii").strip()
                    parts = header.split(":")
                    if len(parts) != 4:
                        raise ValueError(f"不正なヘッダー: {header}")
                    command, ext, seconds_str, size_str = parts
                    command = command.upper()
                    ext = ext.lower().lstrip(".")
                    hold_seconds = float(seconds_str)
                    image_size = int(size_str)
                    if command not in ("IMAGE", "FRAME"):
                        raise ValueError(f"不明なコマンド: {command}")
                    if ext not in ("png", "jpg", "jpeg"):
                        raise ValueError(f"未対応の形式: {ext}")
                    if not math.isfinite(hold_seconds) or hold_seconds < 0:
                        raise ValueError("表示時間は有限の0以上にしてください")
                    if not 1 <= image_size <= MAX_IMAGE_SIZE:
                        raise ValueError(f"画像サイズが不正です: {image_size}")
                    image_data = recv_exact(conn, image_size)
                    manager.show_frame(image_data, hold_seconds,
                                       persistent_zero=(command == "FRAME"))
                    # 従来 IMAGE は指定期間だけ次の IMAGE を受け付けない。
                    # FRAME は待たず、次の画像へ即時切替可能にする。
                    if command == "IMAGE" and hold_seconds > 0:
                        manager.shutdown_event.wait(hold_seconds)
                        manager.expire_frame_if_needed()
                except Exception as e:
                    print(f"[SOCKET CLIENT ERROR] {e}")
    finally:
        sock.close()
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)
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
    print(f"[LCD MIRROR] Watching PNG: {LCD_MIRROR_PATH}")
    idle_buttons = IdleRobotButtonMonitor(manager.shutdown_event, manager.show_transition)
    idle_buttons.start()

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
        idle_buttons.stop()

        log_thread.join(timeout=2.0)

        try:
            display_off()

        except Exception as e:
            print(f"[LCD OFF ERROR] {e}")

        print("[DISPLAY MANAGER] 終了")


if __name__ == "__main__":
    main()