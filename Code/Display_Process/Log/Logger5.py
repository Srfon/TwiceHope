import os
import time
import socket
import math
import tempfile
from pathlib import Path

# display_manager5.py と同じ値にすること (環境変数 DISPLAY_SOCKET_PATH で上書き可)
SOCKET_PATH = os.environ.get("DISPLAY_SOCKET_PATH", "/tmp/display.sock")

LOG_PATH = "/home/trin1/RobCup_V3/Display_Process/Log/system.log"


# Web/LCD共有画像の保存先。RAM上の共有領域なのでSDカードに繰り返し書き込まない。
# /tmp は systemd PrivateTmp により他プロセスから見えないことがあるため使用しない。
LCD_MIRROR_PATH = Path(os.environ.get("LCD_MIRROR_PATH", "/dev/shm/robocup_lcd_web.png"))
MAX_MIRROR_SIZE = 10_000_000


def save_display_mirror(image_data: bytes) -> Path:
    """Web表示データから作ったPNGを原子的に保存する（ソケット送信不要）。

    受信側は保存後の完成PNGだけを読み込む。途中書き込みを見せない。
    """
    if not isinstance(image_data, (bytes, bytearray, memoryview)):
        raise TypeError("image_data はPNGのbytesデータにしてください")
    image_data = bytes(image_data)
    if not 1 <= len(image_data) <= MAX_MIRROR_SIZE:
        raise ValueError("画像データサイズが不正です")
    if not image_data.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("PNG形式の画像データを指定してください")
    LCD_MIRROR_PATH.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        # 同じディレクトリ内で保存 -> os.replace で原子的に差し替え
        with tempfile.NamedTemporaryFile(
            mode="wb", prefix=".lcd_mirror_", suffix=".part",
            dir=str(LCD_MIRROR_PATH.parent), delete=False
        ) as f:
            temporary = Path(f.name)
            f.write(image_data)
        # 別ユーザーで起動したLCDサービスでも読み取れるようにする
        temporary.chmod(0o644)
        os.replace(temporary, LCD_MIRROR_PATH)
        return LCD_MIRROR_PATH
    finally:
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


def logger(tag: str, message: str) -> None:
    """ログファイルへメッセージを書き込む。"""
    line = f"{tag.upper()}: {message}\n"

    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)
    except OSError as e:
        print(f"[ERROR] Log write failed: {e}")


def send_display_image(path: str, display_seconds: float = 3) -> None:
    """画像をディスプレイマネージャーへ送信する。"""

    if not os.path.isfile(path):
        raise FileNotFoundError(path)

    if not math.isfinite(display_seconds) or display_seconds < 0:
        raise ValueError("display_seconds は有限の0以上の値にしてください")

    ext = os.path.splitext(path)[1].lower().lstrip(".")

    if ext == "jpeg":
        ext = "jpg"

    if ext not in ("png", "jpg"):
        raise ValueError("PNGまたはJPEG画像を指定してください")

    with open(path, "rb") as f:
        image_data = f.read()

    header = (
        f"IMAGE:{ext}:{display_seconds:g}:{len(image_data)}\n"
    ).encode("ascii")

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(SOCKET_PATH)
            sock.sendall(header)
            sock.sendall(image_data)

    except FileNotFoundError as e:
        raise RuntimeError(
            f"ソケットが見つかりません: {SOCKET_PATH}。"
            "display_manager5.py が起動しているか、"
            "systemd の PrivateTmp が有効でないか確認してください。"
        ) from e

    except PermissionError as e:
        raise RuntimeError(
            f"ソケットへ接続する権限がありません: {SOCKET_PATH}。"
            "ls -l でパーミッションを確認してください。"
        ) from e

    except ConnectionRefusedError as e:
        raise RuntimeError(
            "ディスプレイマネージャーへの接続が拒否されました。"
        ) from e

    print(f"[SEND] {path}: {len(image_data)} bytes")


def send_display_frame(
    image_data: bytes,
    image_format: str = "jpg",
    hold_seconds: float = 1.0,
    timeout: float = 0.5,
) -> None:
    """画像バイト列を非ブロッキング表示要求(FRAME)として送信する。

    hold_seconds=0 は次のフレーム/画像が来るまで表示を維持する。
    LCD側の描画完了を待つAPIではない。大量連続送信しないよう呼出元でFPSを制限する。
    """
    if not isinstance(image_data, (bytes, bytearray, memoryview)):
        raise TypeError("image_data はbytes系データにしてください")
    ext = image_format.lower().lstrip(".")
    if ext == "jpeg":
        ext = "jpg"
    if ext not in ("png", "jpg"):
        raise ValueError("PNGまたはJPEG画像を指定してください")
    if not math.isfinite(hold_seconds) or hold_seconds < 0:
        raise ValueError("hold_seconds は有限の0以上の値にしてください")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout は有限の正の値にしてください")
    if not 1 <= len(image_data) <= 10_000_000:
        raise ValueError("画像サイズは1〜10000000バイトにしてください")

    header = f"FRAME:{ext}:{hold_seconds:g}:{len(image_data)}\n".encode("ascii")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(SOCKET_PATH)
        sock.sendall(header)
        sock.sendall(image_data)


def main() -> None:
    """動作確認。"""

    for i in range(10): 
        time.sleep(0.5)
        logger("NOTICE", f"Test message2 {i}")
    print("[INFO] Log written")

    omega = (
        "/home/trin1/RobCup_V3/Display_Process/Png/omega.png"
    )
    psi = (
        "/home/trin1/RobCup_V3/Display_Process/Png/psi.png"
    )

    try:
        logger("INFO", "Displaying image")
        for i in range(10):
            send_display_image(omega, display_seconds=0.5)
            send_display_image(psi, display_seconds=0.5)

    except (OSError, RuntimeError, ValueError) as e:
        logger("ERROR", str(e))
        print(f"[ERROR] {e}")


if __name__ == "__main__":
    main()