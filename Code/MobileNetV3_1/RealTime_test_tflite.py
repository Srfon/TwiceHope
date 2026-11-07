"""
camera_inference_tflite.py
==========================
TFLite モデル版 リアルタイム推論

操作：
  s : 推論
  q : 終了ss
"""

import argparse
import cv2
import numpy as np
import tensorflow as tf
from pathlib import Path
import time

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

    # ★ int8モデルの場合：float32 → int8 に変換
    if input_details[0]["dtype"] == np.int8:
        scale, zero = input_details[0]["quantization"]
        inp = np.clip(
            np.round(inp / scale + zero), -128, 127
        ).astype(np.int8)

    interpreter.set_tensor(input_details[0]["index"], inp)
    interpreter.invoke()
    out = interpreter.get_tensor(output_details[0]["index"]).copy()

    # ★ int8モデルの場合：int8 → float32 に逆変換
    if output_details[0]["dtype"] == np.int8:
        scale, zero = output_details[0]["quantization"]
        out = (out.astype(np.float32) - zero) * scale

    probs = out[0]
    idx   = int(np.argmax(probs))
    return class_names[idx], float(probs[idx]), probs


# ============================
# 描画
# ============================
def draw_result(frame, bbox, label, confidence, probs):

    color_map = {
        "H":       (0, 255, 0),
        "S":       (255, 128, 0),
        "U":       (0, 128, 255),
        "unknown": (0, 0, 255),
    }

    color = color_map.get(label, (255, 255, 255))

    x1, y1, x2, y2 = bbox

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    text = f"{label} {confidence*100:.1f}%"

    cv2.putText(
        frame,
        text,
        (x1, max(y1 - 10, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2
    )

    # 確率バー
    bar_x = 10
    bar_y = 10
    bar_w = 160
    bar_h = 18

    for i, (cls, prob) in enumerate(zip(class_names, probs)):

        by = bar_y + i * (bar_h + 4)

        filled = int(np.clip(bar_w * prob, 0, bar_w))

        c = color_map.get(cls, (200, 200, 200))

        cv2.rectangle(
            frame,
            (bar_x, by),
            (bar_x + bar_w, by + bar_h),
            (60, 60, 60),
            -1
        )

        cv2.rectangle(
            frame,
            (bar_x, by),
            (bar_x + filled, by + bar_h),
            c,
            -1
        )

        cv2.putText(
            frame,
            f"{cls}: {prob*100:.1f}%",
            (bar_x + bar_w + 6, by + bar_h - 3),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

    return frame


# ============================
# メイン
# ============================
def main():

    interpreter, input_details, output_details = \
        load_tflite_model(args.model)

    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        raise RuntimeError("カメラを開けません")

    print("カメラ起動完了")
    print("s : 推論")
    print("q : 終了")

    last_result_frame = None

    while True:

        ret, frame = cap.read()

        if not ret:
            break

        display = frame.copy()

        # リアルタイムbbox
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

        if len(contours) > 0:

            cnt = max(contours, key=cv2.contourArea)

            x, y, w, h = cv2.boundingRect(cnt)

            cv2.rectangle(
                display,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                1
            )

        if last_result_frame is not None:
            cv2.addWeighted(
                last_result_frame,
                0.35,
                display,
                0.65,
                0,
                display
            )

        cv2.putText(
            display,
            "s:infer  q:quit",
            (display.shape[1] - 180, display.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1
        )

        cv2.imshow("TFLite Inference", display)

        key = cv2.waitKey(1) & 0xFF

        # 推論
        if key == ord('s'):

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

                print(f"Take {time.time()-start_time} sec")

                print("\n" + "=" * 40)
                print(f"予測: {label}")
                print(f"信頼度: {confidence*100:.1f}%")

                for cls, p in zip(class_names, probs):
                    print(f"{cls:>8}: {p*100:5.1f}%")

                print("=" * 40)

                last_result_frame = draw_result(
                    frame.copy(),
                    bbox,
                    label,
                    confidence,
                    probs
                )

        # 終了
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    print("終了しました")


if __name__ == "__main__":
    main()