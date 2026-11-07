"""
convert_to_tflite_int8.py
=========================
train_v1.py で作成した .keras モデルを
Raspberry Pi 5 向け int8 完全量子化 TFLite に変換する。

【キャリブレーションデータ：val vs 未知データの違い】
─────────────────────────────────────────────────────
● val データ（dataset_V3_output/val）を使う場合
    メリット：
      - モデルが学習時に見た分布と同じ → 値域推定が安定
      - 量子化後の精度低下が最小になりやすい
      - クラスバランスが整っている
    デメリット：
      - 実戦カメラ映像と撮影条件が異なる場合
        量子化後に実戦精度が落ちる可能性がある
    → 推奨：データセットと実戦環境の条件が近いとき

● 未知データ（実戦カメラから収集した生 ROI）を使う場合
    メリット：
      - 推論で実際に入力される画像の値域を直接計測できる
      - val と実戦の乖離が大きい場合に実戦精度が上がることがある
    デメリット：
      - 枚数が少ないと値域推定が不安定になり精度が下がる
      - クラスバランスが崩れやすい
    → 推奨：照明・カメラ・距離が学習時と大きく異なる実戦環境

● 混在（val + 未知データ）
    → 両方の良いとこ取り。最も安定しやすい。
    → --mix_calib --calib_dir で指定
─────────────────────────────────────────────────────

【出力ファイル】
  models/final_v1_float32.tflite  比較用 float32
  models/final_v1_int8.tflite     int8 完全量子化（RasPi5 向け）

使い方：
  # val データでキャリブレーション（デフォルト・推奨）
  python convert_to_tflite_int8.py

  # 未知データでキャリブレーション
  python convert_to_tflite_int8.py --calib_dir path/to/real_roi

  # val + 未知データを混在（最安定）
  python convert_to_tflite_int8.py --mix_calib --calib_dir path/to/real_roi

  # キャリブ枚数を増やして精度低下を抑える
  python convert_to_tflite_int8.py --num_calib 500
"""

import argparse
import numpy as np
import tensorflow as tf
from pathlib import Path

# ============================
# 引数
# ============================
parser = argparse.ArgumentParser()
parser.add_argument("--model",     default="models/final_v1.keras")
parser.add_argument("--val_dir",   default="dataset_V3_output/val",
                    help="val データディレクトリ（デフォルトのキャリブ元）")
parser.add_argument("--calib_dir", default=None,
                    help="未知データディレクトリ（指定時はこちらを優先）")
parser.add_argument("--mix_calib", action="store_true",
                    help="val + 未知データを混在してキャリブレーション")
parser.add_argument("--num_calib", type=int, default=300,
                    help="キャリブレーション画像の合計枚数（200〜500推奨）")
parser.add_argument("--out_dir",   default="models")
args = parser.parse_args()

IMG_SIZE    = 96
class_names = ["H", "S", "U", "unknown"]
NUM_CLASSES = len(class_names)
OUT_DIR     = Path(args.out_dir)
OUT_DIR.mkdir(exist_ok=True)


# ============================
# 画像パス収集
# ============================
def collect_paths(data_dir: str, class_balanced: bool = True) -> list:
    """
    class_balanced=True  : H/S/U/unknown サブディレクトリから均等収集
    class_balanced=False : ディレクトリ以下を再帰的にフラット収集
    """
    p = Path(data_dir)
    if not p.exists():
        raise FileNotFoundError(f"ディレクトリが見つかりません: {p}")

    if class_balanced:
        paths = []
        for cls in class_names:
            d = p / cls
            if d.exists():
                paths += list(d.glob("*.png")) + list(d.glob("*.jpg"))
        return paths
    else:
        return list(p.rglob("*.png")) + list(p.rglob("*.jpg"))


# ============================
# キャリブレーションジェネレータ
# ============================
def make_calibration_generator(num_samples: int):
    """
    --calib_dir / --mix_calib に応じてパスを選択し
    train_v1.py と完全一致の前処理でジェネレータを返す。

    前処理：
      grayscale → [0,1] float32 → 3ch 複製 → [1,H,W,3]
      モデル内 Rescaling(2,-1) が [-1,1] に変換
    """
    # ── パス収集 ──────────────────────────────────
    if args.mix_calib and args.calib_dir:
        val_paths   = collect_paths(args.val_dir,   class_balanced=True)
        unk_paths   = collect_paths(args.calib_dir, class_balanced=False)
        all_paths   = val_paths + unk_paths
        print(f"  キャリブ元 : val {len(val_paths)} 枚 + "
              f"未知 {len(unk_paths)} 枚 = 合計 {len(all_paths)} 枚")
    elif args.calib_dir:
        all_paths = collect_paths(args.calib_dir, class_balanced=False)
        print(f"  キャリブ元 : 未知データ {len(all_paths)} 枚")
    else:
        all_paths = collect_paths(args.val_dir, class_balanced=True)
        print(f"  キャリブ元 : val データ {len(all_paths)} 枚")

    if not all_paths:
        raise ValueError("キャリブレーション画像が見つかりません")

    rng      = np.random.default_rng(42)
    selected = [all_paths[i]
                for i in rng.permutation(len(all_paths))[:min(num_samples,
                                                              len(all_paths))]]
    print(f"  使用枚数   : {len(selected)} 枚")

    def generator():
        for p in selected:
            raw = tf.io.read_file(str(p))
            img = tf.image.decode_image(raw, channels=1,
                                        expand_animations=False)
            img = tf.image.resize(img, [IMG_SIZE, IMG_SIZE])
            img = tf.cast(img, tf.float32) / 255.0   # [0,1]
            img = tf.image.grayscale_to_rgb(img)      # [H,W,3]
            img = tf.expand_dims(img, 0)              # [1,H,W,3]
            yield [img]

    return generator


# ============================
# float32 TFLite 変換（比較用）
# ============================
def convert_float32(keras_model) -> bytes:
    print("\n" + "="*55)
    print("float32 TFLite 変換（比較用）")
    print("="*55)
    converter        = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    tflite_bytes     = converter.convert()
    out_path         = OUT_DIR / (Path(args.model).stem + "_float32.tflite")
    out_path.write_bytes(tflite_bytes)
    print(f"  保存先 : {out_path}")
    print(f"  サイズ : {len(tflite_bytes)/1024/1024:.2f} MB")
    return tflite_bytes


# ============================
# int8 完全量子化 TFLite 変換
# ============================
def convert_int8(keras_model, calib_gen) -> tuple:
    """
    完全 int8 量子化：重み・活性化・入出力すべて int8。

    入出力を int8 に固定する理由：
      float32 入出力のままだと推論のたびに
      float32→int8→float32 変換コストが発生する。
      入出力も int8 にするとこのオーバーヘッドを排除でき
      RasPi5 の ARM Cortex-A76 SIMD 命令を最大活用できる。
      推論スクリプト側で float→int8 スケーリングが必要（後述）。
    """
    print("\n" + "="*55)
    print("int8 完全量子化 TFLite 変換")
    print("="*55)

    converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    converter.optimizations                  = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset         = calib_gen
    converter.target_spec.supported_ops      = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8
    ]
    converter.inference_input_type           = tf.int8
    converter.inference_output_type          = tf.int8

    print("  キャリブレーション実行中（数分かかる場合があります）...")
    tflite_bytes = converter.convert()

    out_path = OUT_DIR / (Path(args.model).stem + "_int8.tflite")
    out_path.write_bytes(tflite_bytes)
    print(f"  保存先 : {out_path}")
    print(f"  サイズ : {len(tflite_bytes)/1024/1024:.2f} MB")
    return tflite_bytes, str(out_path)


# ============================
# TFLite 推論ヘルパー
# ============================
def run_tflite(tflite_bytes: bytes,
               images: np.ndarray,
               is_int8: bool) -> np.ndarray:
    """
    images : [N, H, W, 3] float32 [0,1]
    is_int8: True のとき入出力を int8 にスケーリング
    """
    interpreter = tf.lite.Interpreter(model_content=tflite_bytes)
    interpreter.allocate_tensors()
    inp_d = interpreter.get_input_details()[0]
    out_d = interpreter.get_output_details()[0]

    all_probs = []
    for img in images:
        inp = img[np.newaxis].astype(np.float32)   # [1,H,W,3]
        if is_int8:
            sc, zp = inp_d["quantization"]
            inp = np.clip(np.round(inp / sc + zp),
                          -128, 127).astype(np.int8)

        interpreter.set_tensor(inp_d["index"], inp)
        interpreter.invoke()
        out = interpreter.get_tensor(out_d["index"]).copy()

        if is_int8:
            sc, zp = out_d["quantization"]
            out = (out.astype(np.float32) - zp) * sc

        all_probs.append(out[0])
    return np.array(all_probs)   # [N, 4]


# ============================
# 前処理（評価・推論共通）
# ============================
def load_image(img_path: str) -> np.ndarray:
    """grayscale PNG → [H,W,3] float32 [0,1]"""
    raw = tf.io.read_file(img_path)
    img = tf.image.decode_image(raw, channels=1, expand_animations=False)
    img = tf.image.resize(img, [IMG_SIZE, IMG_SIZE])
    img = tf.cast(img, tf.float32) / 255.0
    img = tf.image.grayscale_to_rgb(img)
    return img.numpy()   # [H,W,3]


# ============================
# 精度評価（Confusion Matrix 付き）
# ============================
def evaluate(tflite_bytes: bytes,
             data_dir: str,
             is_int8: bool,
             label: str) -> float:
    print(f"\n{'='*55}")
    print(f"精度評価: {label}")
    print("="*55)

    images_list, labels_list = [], []
    for cls_idx, cls in enumerate(class_names):
        cls_dir = Path(data_dir) / cls
        if not cls_dir.exists():
            continue
        for p in list(cls_dir.glob("*.png")) + list(cls_dir.glob("*.jpg")):
            images_list.append(load_image(str(p)))
            labels_list.append(cls_idx)

    images = np.array(images_list, dtype=np.float32)
    labels = np.array(labels_list)
    print(f"  評価画像 : {len(images)} 枚")

    probs = run_tflite(tflite_bytes, images, is_int8=is_int8)
    preds = np.argmax(probs, axis=1)
    acc   = float(np.mean(preds == labels))
    print(f"  Accuracy : {acc*100:.2f}%")

    # Confusion Matrix
    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=int)
    for t, p in zip(labels, preds):
        cm[t][p] += 1
    col_w = 11
    print(f"\n{'true \\ pred':>{col_w}}" +
          "".join(f"{c:>{col_w}}" for c in class_names))
    print("-" * (col_w * (NUM_CLASSES + 1)))
    for i, name in enumerate(class_names):
        print(f"{name:>{col_w}}" +
              "".join(f"{cm[i][j]:>{col_w}}" for j in range(NUM_CLASSES)))

    # Per-class 指標
    print(f"\n{'class':>10}  {'Recall':>8}  {'Precision':>10}"
          f"  {'F1':>8}  {'Support':>8}")
    print("-"*55)
    f1s = []
    for i, name in enumerate(class_names):
        sup  = int(cm[i].sum())
        rec  = cm[i][i] / sup             if sup            > 0 else 0.0
        prec = cm[i][i] / cm[:, i].sum() if cm[:, i].sum() > 0 else 0.0
        f1   = 2*prec*rec/(prec+rec)     if (prec+rec)     > 0 else 0.0
        f1s.append(f1)
        print(f"{name:>10}  {rec:>8.3f}  {prec:>10.3f}"
              f"  {f1:>8.3f}  {sup:>8}")
    print(f"\n  Macro F1 : {np.mean(f1s):.4f}")
    return acc


# ============================
# RasPi5 推論サンプルを出力
# ============================
def print_raspi_sample(int8_path: str):
    stem = Path(int8_path).name
    print(f"""
{"="*55}
RasPi5 での推論サンプルコード（camera_inference.py に組み込む場合）
{"="*55}
インストール（RasPi5 上で）:
  pip install tflite-runtime opencv-python-headless numpy

推論スクリプト例:
─────────────────────────────────────────
import numpy as np
import tflite_runtime.interpreter as tflite
import cv2

IMG_SIZE    = {IMG_SIZE}
class_names = {class_names}

interpreter = tflite.Interpreter(model_path="{stem}")
interpreter.allocate_tensors()
inp_d = interpreter.get_input_details()[0]
out_d = interpreter.get_output_details()[0]
inp_scale, inp_zero = inp_d["quantization"]
out_scale, out_zero = out_d["quantization"]

def infer(roi_gray_96x96: np.ndarray):
    \"\"\"roi_gray_96x96: uint8 grayscale ndarray\"\"\"
    img = roi_gray_96x96.astype(np.float32) / 255.0
    img = np.stack([img]*3, axis=-1)              # [H,W,3]
    inp = img[np.newaxis]                         # [1,H,W,3]

    # float32 → int8
    inp_int8 = np.clip(
        np.round(inp / inp_scale + inp_zero), -128, 127
    ).astype(np.int8)

    interpreter.set_tensor(inp_d["index"], inp_int8)
    interpreter.invoke()
    out = interpreter.get_tensor(out_d["index"]).copy()

    # int8 → float32
    probs = (out.astype(np.float32) - out_zero) * out_scale
    idx   = int(np.argmax(probs))
    return class_names[idx], float(probs[0][idx])

label, conf = infer(your_96x96_gray_roi)
print(f"{{label}}  {{conf*100:.1f}}%")
─────────────────────────────────────────
""")


# ============================
# メイン
# ============================
def main():
    keras_path = Path(args.model)
    if not keras_path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {keras_path}")

    print("="*55)
    print("keras → TFLite int8 変換（RasPi5 向け）")
    print("="*55)
    print(f"  変換元     : {keras_path}")
    if args.mix_calib and args.calib_dir:
        mode = f"val + 未知データ混在（{args.calib_dir}）"
    elif args.calib_dir:
        mode = f"未知データのみ（{args.calib_dir}）"
    else:
        mode = "val データのみ（デフォルト）"
    print(f"  キャリブ   : {mode}")
    print(f"  キャリブ枚数: {args.num_calib} 枚")

    print("\nモデルロード中...")
    keras_model = tf.keras.models.load_model(
        str(keras_path), compile=False
    )

    # ── 変換 ──────────────────────────────────────
    f32_bytes            = convert_float32(keras_model)
    calib_gen            = make_calibration_generator(args.num_calib)
    int8_bytes, int8_path = convert_int8(keras_model, calib_gen)

    # ── 精度比較 ───────────────────────────────────
    print("\n" + "="*55)
    print("精度比較（val データ全件）")
    print("="*55)
    acc_f32  = evaluate(f32_bytes,  args.val_dir,
                        is_int8=False, label="float32 TFLite")
    acc_int8 = evaluate(int8_bytes, args.val_dir,
                        is_int8=True,  label="int8 TFLite")

    # ── サマリ ─────────────────────────────────────
    f32_mb  = len(f32_bytes)  / 1024 / 1024
    int8_mb = len(int8_bytes) / 1024 / 1024
    drop    = acc_f32 - acc_int8

    print("\n" + "="*55)
    print("変換結果サマリ")
    print("="*55)
    print(f"  {'':18}  {'float32':>10}  {'int8':>10}")
    print(f"  {'モデルサイズ':18}  {f32_mb:>9.2f}MB  {int8_mb:>9.2f}MB")
    print(f"  {'Accuracy':18}  {acc_f32*100:>9.2f}%  {acc_int8*100:>9.2f}%")
    print(f"  {'精度低下':18}  {'—':>10}  {drop*100:>+9.2f}%")
    print(f"  {'圧縮率':18}  {'—':>10}  {f32_mb/int8_mb:>9.1f}x")
    print(f"\n  int8 モデル保存先: {int8_path}")

    if drop > 0.02:
        print(f"""
[!] 精度低下 {drop*100:.1f}% → 以下を試してください：

  1. キャリブ枚数を増やす（最も効果的）
       python convert_to_tflite_int8.py --num_calib 500

  2. 未知データ（実戦 ROI）を混在させる
       python convert_to_tflite_int8.py \\
           --mix_calib --calib_dir path/to/real_roi

  3. 未知データのみでキャリブ（実戦と学習の乖離が大きい場合）
       python convert_to_tflite_int8.py --calib_dir path/to/real_roi
""")
    else:
        print(f"\n  精度低下 {drop*100:.2f}% → 許容範囲内 ✓")

    print_raspi_sample(int8_path)
    print("convert_to_tflite_int8 finished")


if __name__ == "__main__":
    main()
