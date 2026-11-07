import os
import time
import socket

# display_manager.py と同じ値にすること (環境変数 DISPLAY_SOCKET_PATH で上書き可)
SOCKET_PATH = os.environ.get("DISPLAY_SOCKET_PATH", "/tmp/display.sock")

LOG_PATH = "/home/trin1/RobCup_V3/Display_Process/Log/system.log"


def logger(tag: str, message: str) -> None:
    """ログファイルへメッセージを書き込む。"""
    line = f"{tag.upper()}: {message}\n"

    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line)
    except OSError as e:
        print(f"[ERROR] Log write failed: {e}")


def send_display_image(path: str, display_seconds: int = 3) -> None:
    """画像をディスプレイマネージャーへ送信する。"""

    if not os.path.isfile(path):
        raise FileNotFoundError(path)

    if display_seconds < 0:
        raise ValueError("display_seconds は0以上にしてください")

    ext = os.path.splitext(path)[1].lower().lstrip(".")

    if ext == "jpeg":
        ext = "jpg"

    if ext not in ("png", "jpg"):
        raise ValueError("PNGまたはJPEG画像を指定してください")

    with open(path, "rb") as f:
        image_data = f.read()

    header = (
        f"IMAGE:{ext}:{int(display_seconds)}:{len(image_data)}\n"
    ).encode("ascii")

    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.connect(SOCKET_PATH)
            sock.sendall(header)
            sock.sendall(image_data)

    except FileNotFoundError as e:
        raise RuntimeError(
            f"ソケットが見つかりません: {SOCKET_PATH}。"
            "display_manager.py が起動しているか、"
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