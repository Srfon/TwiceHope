"""
camera_inference_tflite.py
==========================
TFLite モデル版 リアルタイム推論 (ヘッドレス / ターミナル入出力版)

cv2.imshow() / cv2.waitKey() はQtのxcbプラットフォームプラグインに依存して
おり、ディスプレイの無い環境 (SSH接続のみ、X転送無し等) では

    qt.qpa.xcb: could not connect to display
    qt.qpa.plugin: Could not load the Qt platform plugin "xcb"

のようなエラーで起動できない。本バージョンではGUI描画を一切行わず、
標準入力からのコマンド ('s' + Enter で推論、'q' + Enter で終了) のみで
動作するようにした。カメラのキャプチャ自体・ROI抽出(閾値処理/輪郭検出)は
GUIを使わないOpenCVのAPIのみなので、ヘッドレス環境でも問題なく動く。

操作 (ターミナル上で入力してEnter)：
  s : 推論
  q : 終了
"""

import argparse
import select
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import tensorflow as tf

# ============================
# 引数
# ============================
parser = argparse.ArgumentParser()
parser.add_argument(
    "--model",
    default="/home/trin1/RobCup_V3/MobileNetV3_1/models/final_v1_int8.tflite",
    help="TFLiteモデルのパス"
)
parser.add_argument(
    "--camera",
    type=int,
    default=0,
    help="カメラ番号"
)
args = parser.parse_args()

IMG_SIZE = 96
class_names = ["H", "S", "U", "unknown"]


# ============================
# TFLite モデルロード
# ============================
def load_tflite_model(model_path):
    path = Path(model_path)

    if not path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {path}")

    print(f"TFLiteモデルをロード中: {path}")

    interpreter = tf.lite.Interpreter(model_path=str(path))
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    print("モデルロード完了")
    print("入力:", input_details)
    print("出力:", output_details)

    return interpreter, input_details, output_details


# ============================
# ROI切り出し
# ============================
def extract_roi(frame, margin=5):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    _, th = cv2.threshold(
        gray,
        0,
        255,
        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )

    contours, _ = cv2.findContours(
        th,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if len(contours) == 0:
        return None, None

    cnt = max(contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(cnt)

    x1 = max(x - margin, 0)
    y1 = max(y - margin, 0)
    x2 = min(x + w + margin, frame.shape[1])
    y2 = min(y + h + margin, frame.shape[0])

    roi = gray[y1:y2, x1:x2]

    roi_resized = cv2.resize(
        roi,
        (IMG_SIZE, IMG_SIZE),
        interpolation=cv2.INTER_AREA
    )

    return roi_resized, (x1, y1, x2, y2)


# ============================
# 前処理
# ============================
def preprocess(roi_gray):

    img = roi_gray.astype(np.float32) / 255.0

    img = np.stack([img, img, img], axis=-1)

    img = np.expand_dims(img, axis=0)

    return img.astype(np.float32)


# ============================
# TFLite 推論
# ============================
def predict(interpreter,
            input_details,
            output_details,
            roi_gray):

    inp = preprocess(roi_gray)  # [1,H,W,3] float32 [0,1]

    # int8モデルの場合：float32 → int8 に変換
    if input_details[0]["dtype"] == np.int8:
        scale, zero = input_details[0]["quantization"]
        inp = np.clip(
            np.round(inp / scale + zero), -128, 127
        ).astype(np.int8)

    interpreter.set_tensor(input_details[0]["index"], inp)
    interpreter.invoke()
    out = interpreter.get_tensor(output_details[0]["index"]).copy()

    # int8モデルの場合：int8 → float32 に逆変換
    if output_details[0]["dtype"] == np.int8:
        scale, zero = output_details[0]["quantization"]
        out = (out.astype(np.float32) - zero) * scale

    probs = out[0]
    idx = int(np.argmax(probs))
    return class_names[idx], float(probs[idx]), probs


# ============================
# 結果をターミナルへ整形表示
# ============================
def print_result(label, confidence, probs, elapsed_sec):

    print(f"Take {elapsed_sec:.3f} sec")
    print("\n" + "=" * 40)
    print(f"予測: {label}")
    print(f"信頼度: {confidence * 100:.1f}%")

    for cls, p in zip(class_names, probs):
        print(f"{cls:>8}: {p * 100:5.1f}%")

    print("=" * 40)


# ============================
# 標準入力の非ブロッキング読み取り
#
# select()でstdinにデータが来ているかをブロックせずに確認し、
# 来ていれば1行読み取って先頭文字を返す。無ければNone。
# ============================
def read_stdin_command(timeout=0.0):

    ready, _, _ = select.select([sys.stdin], [], [], timeout)

    if not ready:
        return None

    line = sys.stdin.readline()

    if line == "":
        # EOF (パイプ入力が終わった等)
        return "q"

    line = line.strip()

    return line[0].lower() if line else None


# ============================
# メイン
# ============================
def main():

    interpreter, input_details, output_details = \
        load_tflite_model(args.model)

    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError("カメラを開けません")

    print("カメラ起動完了 (ヘッドレスモード / GUI表示なし)")
    print("s + Enter : 推論")
    print("q + Enter : 終了")

    try:
        while True:

            ret, frame = cap.read()

            if not ret:
                print("フレーム取得に失敗しました")
                break

            cmd = read_stdin_command()

            if cmd == 's':

                roi, bbox = extract_roi(frame)

                start_time = time.time()

                if roi is None:
                    print("ブロブ未検出")

                else:
                    label, confidence, probs = predict(
                        interpreter,
                        input_details,
                        output_details,
                        roi
                    )

                    elapsed = time.time() - start_time

                    print_result(label, confidence, probs, elapsed)

            elif cmd == 'q':
                break

            # cv2.waitKey相当のフレームレート制御 (busy-loop防止)
            time.sleep(0.01)

    finally:
        cap.release()
        print("終了しました")


if __name__ == "__main__":
    main()