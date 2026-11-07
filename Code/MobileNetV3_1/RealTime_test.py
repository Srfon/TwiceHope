"""
camera_inference.py
===================
リアルタイムカメラ映像から ROI を切り出し、
MobileNetV3Small モデルで H/S/U/unknown を推論する。

操作：
  s キー : ROI を切り出して推論 + 結果表示
  q キー : 終了

使い方：
  python camera_inference.py
  python camera_inference.py --model models/final_v1.keras
  python camera_inference.py --model models/final_v1.keras --camera 0
"""

import argparse
import cv2
import numpy as np
import tensorflow as tf
from pathlib import Path

# ============================
# 引数
# ============================
parser = argparse.ArgumentParser()
parser.add_argument("--model",  default="models/final_v1.keras",
                    help="推論に使うモデルのパス")
parser.add_argument("--camera", type=int, default=1,
                    help="カメラデバイス番号（デフォルト: 1）")
args = parser.parse_args()

IMG_SIZE    = 96
class_names = ["H", "S", "U", "unknown"]


# ============================
# モデルロード＋コンパイル
# ============================
def load_model(model_path: str) -> tf.keras.Model:
    path = Path(model_path)
    if not path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {path}")
    print(f"モデルをロード中: {path}")
    model = tf.keras.models.load_model(str(path), compile=False)
    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-5),
        loss=tf.keras.losses.CategoricalCrossentropy(),
        metrics=["accuracy"],
    )
    print("モデルロード完了")
    return model


# ============================
# ROI 切り出し
# ============================
def extract_roi(frame, margin=5):
    """
    フレームから最大ブロブの ROI を切り出して 96x96 グレースケールで返す。
    戻り値: (roi_resized, bbox) または (None, None)
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, th = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    contours, _ = cv2.findContours(
        th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours) == 0:
        return None, None

    cnt  = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(cnt)

    x1 = max(x - margin, 0)
    y1 = max(y - margin, 0)
    x2 = min(x + w + margin, frame.shape[1])
    y2 = min(y + h + margin, frame.shape[0])

    roi = gray[y1:y2, x1:x2]
    roi_resized = cv2.resize(roi, (IMG_SIZE, IMG_SIZE),
                             interpolation=cv2.INTER_AREA)
    return roi_resized, (x1, y1, x2, y2)


# ============================
# 前処理（train_v1.py と完全一致）
# ============================
def preprocess(roi_gray: np.ndarray) -> np.ndarray:
    """
    grayscale 96x96 (uint8) → モデル入力 (1, 96, 96, 3) float32 [0,1]
    ※ モデル内の Rescaling(2,-1) で [-1,1] に変換される
    """
    img = roi_gray.astype(np.float32) / 255.0          # [0,1]
    img = np.stack([img, img, img], axis=-1)            # [H,W,3]
    img = np.expand_dims(img, axis=0)                   # [1,H,W,3]
    return img


# ============================
# 推論
# ============================
def predict(model: tf.keras.Model, roi_gray: np.ndarray) -> tuple:
    """
    戻り値:
      label     : 予測クラス名 (str)
      confidence: 最大クラスの確率 (float)
      probs     : 全クラスの確率配列 (np.ndarray)
    """
    inp   = preprocess(roi_gray)
    probs = model.predict(inp, verbose=0)[0]   # shape: (4,)
    idx   = int(np.argmax(probs))
    return class_names[idx], float(probs[idx]), probs


# ============================
# 結果オーバーレイ描画
# ============================
def draw_result(frame, bbox, label, confidence, probs):
    """
    推論結果をフレームに描画する。
    - バウンディングボックス（クラスで色分け）
    - ラベル + 信頼度
    - 全クラスの確率バー
    """
    color_map = {
        "H":       (0,   255,   0),    # 緑
        "S":       (255,  128,   0),   # オレンジ
        "U":       (0,   128, 255),    # 水色
        "unknown": (0,     0, 255),    # 赤
    }
    color = color_map.get(label, (255, 255, 255))

    x1, y1, x2, y2 = bbox
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    # ラベル + 信頼度
    text = f"{label}  {confidence*100:.1f}%"
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
    ty = max(y1 - 8, th + 4)
    cv2.rectangle(frame, (x1, ty - th - 4), (x1 + tw + 4, ty + 4), color, -1)
    cv2.putText(frame, text, (x1 + 2, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

    # 全クラス確率バー（左上）
    bar_x, bar_y = 10, 10
    bar_w, bar_h = 160, 18
    for i, (cls, prob) in enumerate(zip(class_names, probs)):
        by = bar_y + i * (bar_h + 4)
        filled = int(bar_w * prob)
        c = color_map.get(cls, (200, 200, 200))
        cv2.rectangle(frame, (bar_x, by),
                      (bar_x + bar_w, by + bar_h), (60, 60, 60), -1)
        cv2.rectangle(frame, (bar_x, by),
                      (bar_x + filled, by + bar_h), c, -1)
        cv2.putText(frame, f"{cls}: {prob*100:.1f}%",
                    (bar_x + bar_w + 6, by + bar_h - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return frame


# ============================
# メイン
# ============================
def main():
    model = load_model(args.model)

    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError(f"カメラ {args.camera} を開けません")

    print("カメラ起動完了")
    print("  s : 推論実行")
    print("  q : 終了")

    last_result_frame = None   # 最後の推論結果を保持して表示し続ける

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        display = frame.copy()

        # ── 毎フレーム：最大ブロブの bbox をリアルタイム描画 ──
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )
        contours, _ = cv2.findContours(
            th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 1)

        # ── 前回の推論結果テキストを常時表示 ──
        if last_result_frame is not None:
            # 結果オーバーレイを半透明合成
            cv2.addWeighted(last_result_frame, 0.35, display, 0.65, 0, display)

        cv2.putText(display, "s:infer  q:quit",
                    (display.shape[1] - 180, display.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("MobileNetV3 Inference", display)

        key = cv2.waitKey(1) & 0xFF

        # ── s キー：推論 ──
        if key == ord('s'):
            roi, bbox = extract_roi(frame)
            if roi is None:
                print("[!] ブロブが検出されませんでした")
            else:
                label, confidence, probs = predict(model, roi)

                # コンソール出力
                print("\n" + "="*40)
                print(f"  予測     : {label}")
                print(f"  信頼度   : {confidence*100:.1f}%")
                print("  全クラス確率:")
                for cls, p in zip(class_names, probs):
                    bar = "█" * int(p * 20)
                    print(f"    {cls:>8}: {p*100:5.1f}%  {bar}")
                print("="*40)

                # 結果オーバーレイを保持
                last_result_frame = draw_result(
                    frame.copy(), bbox, label, confidence, probs
                )

        # ── q キー：終了 ──
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("終了しました")


if __name__ == "__main__":
    main()