"""
train_v1.py
===========
MobileNetV3Small を ImageNet 重みからゼロ構築。
過去モデルは一切使用しない。

【修正済み問題】
  - include_preprocessing=True + [0,1]入力による val_loss 爆発
    → False に変更し Rescaling レイヤーで train/val を統一
  - val_ds の repeat() なしによる枯渇警告
    → make_ds に repeat() 追加、evaluate 専用に make_ds_eval を分離
  - augment_fn の random_crop で shape 不一致エラー
    → unbatch → map → batch の順に修正済み

構成：
  Step 1  データセット統計
  Step 2  Head のみ学習（backbone 全凍結）
  Step 3  段階的 Fine-tuning（FT_A: 20層 → FT_B: 40層 → FT_C: 全層）
  Step 4  過学習診断レポート
  Step 5  Confusion Matrix

データセット構造：
  dataset_V3_output/
    train/  H/  S/  U/  unknown/
    val/    H/  S/  U/  unknown/

使い方：
  python train_v1.py
  python train_v1.py --unk_weight 0.55
  python train_v1.py --data dataset_V3_output
"""

import argparse
import math
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, Model
from tensorflow.keras.callbacks import EarlyStopping, ModelCheckpoint
from tensorflow.keras.applications import MobileNetV3Small
from pathlib import Path

# ============================
# 引数
# ============================
parser = argparse.ArgumentParser()
parser.add_argument("--data",       default="dataset_V3_output")
parser.add_argument("--unk_weight", type=float, default=0.65,
                    help="unknown クラスの重み乗数（0.3〜0.8）")
args = parser.parse_args()

IMG_SIZE    = 96
BATCH_SIZE  = 16
class_names = ["H", "S", "U", "unknown"]
NUM_CLASSES = len(class_names)
MODEL_DIR   = Path("models")
MODEL_DIR.mkdir(exist_ok=True)


# ============================
# データセット統計・クラス重み
# ============================
def print_dataset_stats(split_dir: Path, label: str = "") -> dict:
    title = f"データセット統計 [{label}]" if label else "データセット統計"
    print("\n" + "="*55)
    print(title)
    print("="*55)
    counts = {}
    for cls in class_names:
        d = split_dir / cls
        n = len(list(d.glob("*.png"))) if d.exists() else 0
        counts[cls] = n
        print(f"  {cls:>10}: {n:>6} 枚")
    print(f"  {'合計':>10}: {sum(counts.values()):>6} 枚")
    return counts


def compute_class_weight(counts: dict, unk_mult: float) -> dict:
    total = sum(counts.values())
    cw = {i: total / (NUM_CLASSES * counts[cls])
          for i, cls in enumerate(class_names)
          if counts[cls] > 0}
    cw[3] *= unk_mult
    print(f"\nclass_weight (unknown×{unk_mult}):",
          {class_names[k]: f"{v:.3f}" for k, v in cw.items()})
    return cw


# ============================
# Dataset
# ============================
def grayscale_to_rgb(image, label):
    """
    1ch → 3ch 複製 + [0,255] → [0,1] 正規化
    train / val 両方に適用（値域を統一する）
    モデル内の Rescaling で [0,1] → [-1,1] に変換するため
    ここでは [0,1] に留める
    """
    image = tf.cast(image, tf.float32) / 255.0
    image = tf.image.grayscale_to_rgb(image)   # [H,W,1] → [H,W,3]
    return image, label


def augment_fn(image, label):
    """
    実環境に即した Augmentation（train 専用・1枚単位で呼ばれる）
    - 手ブレ・ピンぼけ模倣
    - 明暗・コントラスト変化
    - 位置ズレ・スケールばらつき（検出器のブレ模倣）
    - 文字認識のため左右反転は無効
    """
    # 明暗・コントラスト
    image = tf.image.random_brightness(image, max_delta=0.3)
    image = tf.image.random_contrast(image, lower=0.7, upper=1.4)

    # ランダムクロップ（±8px のズレを模倣）
    pad = 8
    image = tf.image.resize_with_crop_or_pad(
        image, IMG_SIZE + pad * 2, IMG_SIZE + pad * 2
    )
    image = tf.image.random_crop(image, size=[IMG_SIZE, IMG_SIZE, 3])

    # ランダムぼかし（手ブレ・ピンぼけ）
    if tf.random.uniform([]) < 0.4:
        image = tf.nn.avg_pool2d(
            tf.expand_dims(image, 0),
            ksize=3, strides=1, padding='SAME'
        )[0]

    # ガウスノイズ
    if tf.random.uniform([]) < 0.3:
        noise = tf.random.normal(
            shape=tf.shape(image), mean=0.0, stddev=0.04
        )
        image = tf.clip_by_value(image + noise, 0.0, 1.0)

    image = tf.clip_by_value(image, 0.0, 1.0)
    return image, label


def make_ds(split_dir: str, augment: bool = False):
    """
    fit() 用データセット（repeat() あり）
    val_ds にも repeat() をつけることで
    fit() の validation_steps が安定する
    """
    ds = tf.keras.utils.image_dataset_from_directory(
        split_dir,
        image_size=(IMG_SIZE, IMG_SIZE),
        batch_size=BATCH_SIZE,
        color_mode="grayscale",
        label_mode="categorical",
        shuffle=augment,
        seed=42,
    )
    # grayscale → RGB + [0,1] 正規化（train/val 共通）
    ds = ds.map(grayscale_to_rgb, num_parallel_calls=tf.data.AUTOTUNE)
    if augment:
        # unbatch → 1枚ずつ augment → rebatch
        ds = ds.unbatch()
        ds = ds.map(augment_fn, num_parallel_calls=tf.data.AUTOTUNE)
        ds = ds.batch(BATCH_SIZE)
    # repeat() で枯渇警告を解消
    return ds.repeat().prefetch(tf.data.AUTOTUNE)


def make_ds_eval(split_dir: str):
    """
    evaluate() / predict() / Confusion Matrix 専用（repeat なし）
    repeat() があると無限ループするため分離
    """
    ds = tf.keras.utils.image_dataset_from_directory(
        split_dir,
        image_size=(IMG_SIZE, IMG_SIZE),
        batch_size=BATCH_SIZE,
        color_mode="grayscale",
        label_mode="categorical",
        shuffle=False,
        seed=42,
    )
    ds = ds.map(grayscale_to_rgb, num_parallel_calls=tf.data.AUTOTUNE)
    return ds.prefetch(tf.data.AUTOTUNE)


# ============================
# モデル構築（ゼロから）
# ============================
def build_model(num_classes: int = NUM_CLASSES) -> tuple:
    """
    MobileNetV3Small + カスタム Head
    - ImageNet 重みを使用（backbone の特徴抽出能力を最大活用）
    - include_top=False で Head を自前で定義
    - include_preprocessing=False（データ側で [0,1] に統一済み）
    - Rescaling レイヤーで [0,1] → [-1,1] に変換（ImageNet 準拠）
      これにより train/val の前処理が完全に一致する
    """
    base = MobileNetV3Small(
        input_shape=(IMG_SIZE, IMG_SIZE, 3),
        include_top=False,
        weights="imagenet",
        include_preprocessing=False,   # ★ False：二重変換を防ぐ
    )
    base.trainable = False  # 初期は全凍結

    inputs = tf.keras.Input(shape=(IMG_SIZE, IMG_SIZE, 3))
    # ★ [0,1] → [-1,1]（ImageNet の前処理と等価）
    x = layers.Rescaling(scale=2.0, offset=-1.0)(inputs)
    x = base(x, training=False)
    x = layers.GlobalAveragePooling2D()(x)
    x = layers.BatchNormalization()(x)
    x = layers.Dense(128, activation="relu")(x)
    x = layers.Dropout(0.5)(x)
    outputs = layers.Dense(num_classes, activation="softmax")(x)

    return Model(inputs, outputs), base


# ============================
# Cosine Decay with Warmup
# ============================
class WarmupCosineDecay(tf.keras.optimizers.schedules.LearningRateSchedule):
    """
    局所最適脱出に有効なスケジューラ。
    warmup_steps で lr を 0 → peak_lr まで線形増加させ、
    その後 Cosine Decay で min_lr まで減衰させる。
    """
    def __init__(self, peak_lr, total_steps, warmup_steps=100, min_lr=1e-9):
        super().__init__()
        self.peak_lr      = float(peak_lr)
        self.total_steps  = float(total_steps)
        self.warmup_steps = float(warmup_steps)
        self.min_lr       = float(min_lr)

    def __call__(self, step):
        step      = tf.cast(step, tf.float32)
        warmup_lr = self.peak_lr * (step / self.warmup_steps)
        progress  = (step - self.warmup_steps) / \
                    (self.total_steps - self.warmup_steps)
        progress  = tf.clip_by_value(progress, 0.0, 1.0)
        cosine_lr = self.min_lr + 0.5 * (self.peak_lr - self.min_lr) * (
            1.0 + tf.cos(math.pi * progress))
        return tf.where(step < self.warmup_steps, warmup_lr, cosine_lr)

    def get_config(self):
        return {
            "peak_lr":      self.peak_lr,
            "total_steps":  self.total_steps,
            "warmup_steps": self.warmup_steps,
            "min_lr":       self.min_lr,
        }


def make_schedule(peak_lr, epochs, steps_per_epoch, warmup_epochs=3):
    total  = epochs * steps_per_epoch
    warmup = warmup_epochs * steps_per_epoch
    return WarmupCosineDecay(peak_lr, total, warmup, min_lr=peak_lr * 0.01)


# ============================
# コールバック
# ============================
def make_callbacks(name: str, patience_es: int,
                   steps_per_epoch: int, val_steps: int) -> list:
    return [
        EarlyStopping(
            monitor="val_loss",
            patience=patience_es,
            restore_best_weights=True,
            verbose=1,
        ),
        ModelCheckpoint(
            filepath=str(MODEL_DIR / name),
            monitor="val_loss",
            save_best_only=True,
            verbose=1,
        ),
    ]


# ============================
# BN固定・解凍ヘルパー
# ============================
def set_unfreeze(base_model, n_unfreeze):
    """末尾 n_unfreeze 層を解凍。None=全層。BN は常に固定。"""
    base_model.trainable = True
    freeze_until = (
        max(0, len(base_model.layers) - n_unfreeze)
        if n_unfreeze is not None else 0
    )
    for i, layer in enumerate(base_model.layers):
        if isinstance(layer, tf.keras.layers.BatchNormalization):
            layer.trainable = False
        else:
            layer.trainable = (i >= freeze_until)
    n_tr = sum(1 for l in base_model.layers if l.trainable)
    print(f"  解凍層数: {n_tr} / {len(base_model.layers)}  (BN固定)")


# ============================
# モデルロード（Stage 間引き継ぎ用）
# ============================
def load_model_recompile(path: str) -> tuple:
    """
    前 Stage の最良モデルをロードして返す。
    compile は呼び出し側で行う。
    """
    model = tf.keras.models.load_model(
        str(path),
        custom_objects={"WarmupCosineDecay": WarmupCosineDecay},
        compile=False,
    )
    base = next(
        (l for l in model.layers
         if isinstance(l, tf.keras.Model)
         and "mobilenet" in l.name.lower()),
        None,
    )
    if base is None:
        raise RuntimeError("MobileNetV3 レイヤーが見つかりません")
    return model, base


# ============================
# 過学習診断
# ============================
def diagnose_overfitting(history_log: list):
    print("\n" + "="*55)
    print("Step 4: 過学習診断レポート")
    print("="*55)
    print(f"{'Stage':>8}  {'train_acc':>10}  {'val_acc':>10}"
          f"  {'乖離':>8}  {'判定':>14}")
    print("-"*55)
    for stage_name, h in history_log:
        if not h.history.get("accuracy"):
            continue
        tr_acc   = h.history["accuracy"][-1]
        val_acc  = h.history["val_accuracy"][-1]
        tr_loss  = h.history["loss"][-1]
        val_loss = h.history["val_loss"][-1]
        gap      = tr_acc - val_acc
        if gap > 0.10:
            verdict = "⚠ 過学習"
        elif val_loss > tr_loss * 1.5 and gap > 0.05:
            verdict = "△ やや過学習"
        elif val_acc < 0.70 and gap < 0.03:
            verdict = "△ 局所最適の疑い"
        else:
            verdict = "✓ 正常"
        print(f"{stage_name:>8}  {tr_acc:>10.3f}  {val_acc:>10.3f}"
              f"  {gap:>8.3f}  {verdict:>14}")

    print("""
局所最適の疑い（val_acc 停滞・乖離小）の場合：
  → --unk_weight を下げて再実行（例: --unk_weight 0.35）
  → unknown データに H に似たサンプルを追加収集する

過学習（乖離 > 0.10）の場合：
  → Dropout を増やす、データ拡張を強化する
""")


# ============================
# Confusion Matrix
# ============================
def print_confusion_matrix(model, val_ds_eval):
    print("\n" + "="*55)
    print("Step 5: Confusion Matrix (validation set)")
    print("="*55)
    y_true_list, y_pred_list = [], []
    for images, labels in val_ds_eval:
        preds = model.predict(images, verbose=0)
        y_true_list.extend(np.argmax(labels.numpy(), axis=1))
        y_pred_list.extend(np.argmax(preds, axis=1))
    y_true = np.array(y_true_list)
    y_pred = np.array(y_pred_list)

    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=int)
    for t, p in zip(y_true, y_pred):
        cm[t][p] += 1

    col_w = 11
    print(f"{'true \\ pred':>{col_w}}" +
          "".join(f"{c:>{col_w}}" for c in class_names))
    print("-" * (col_w * (NUM_CLASSES + 1)))
    for i, name in enumerate(class_names):
        print(f"{name:>{col_w}}" +
              "".join(f"{cm[i][j]:>{col_w}}" for j in range(NUM_CLASSES)))

    print(f"\n{'class':>10}  {'Recall':>8}  {'Precision':>10}"
          f"  {'F1':>8}  {'Support':>8}")
    print("-"*55)
    recalls, f1s = [], []
    for i, name in enumerate(class_names):
        sup  = int(cm[i].sum())
        rec  = cm[i][i] / sup             if sup            > 0 else 0.0
        prec = cm[i][i] / cm[:, i].sum() if cm[:, i].sum() > 0 else 0.0
        f1   = 2*prec*rec/(prec+rec)     if (prec+rec)     > 0 else 0.0
        recalls.append(rec)
        f1s.append(f1)
        print(f"{name:>10}  {rec:>8.3f}  {prec:>10.3f}"
              f"  {f1:>8.3f}  {sup:>8}")

    acc = float(np.sum(y_true == y_pred) / len(y_true))
    print(f"\n  Accuracy : {acc:.4f}")
    print(f"  Macro F1 : {float(np.mean(f1s)):.4f}")

    unk_rec = recalls[class_names.index("unknown")]
    hsu_avg = float(np.mean(recalls[:3]))
    if unk_rec < 0.65:
        print(f"\n[!] unknown Recall={unk_rec:.3f} → unknown→H 誤分類が継続")
        print(f"    対策: --unk_weight {max(0.3, args.unk_weight - 0.1):.1f} で再実行")
        print("         または unknown に H 字型サンプルを追加収集")
    if hsu_avg < 0.80:
        print(f"\n[!] H/S/U 平均 Recall={hsu_avg:.3f}")
        print(f"    対策: --unk_weight {min(0.7, args.unk_weight + 0.1):.1f} で再実行")


# ============================
# メイン
# ============================
def main():
    output_dir = Path(args.data)
    train_dir  = output_dir / "train"
    val_dir    = output_dir / "val"

    # ---------- Step 1 ----------
    counts_train = print_dataset_stats(train_dir, label="train")
    counts_val   = print_dataset_stats(val_dir,   label="val")
    cw = compute_class_weight(counts_train, args.unk_weight)

    # ---------- Dataset ----------
    train_ds    = make_ds(str(train_dir), augment=True)   # fit 用（repeat あり）
    val_ds      = make_ds(str(val_dir),   augment=False)  # fit 用（repeat あり）
    val_ds_eval = make_ds_eval(str(val_dir))              # evaluate / CM 用（repeat なし）

    n_train         = sum(counts_train.values())
    n_val           = sum(counts_val.values())
    steps_per_epoch = max(1, n_train // BATCH_SIZE)
    val_steps       = max(1, n_val   // BATCH_SIZE)

    print(f"\n  steps_per_epoch : {steps_per_epoch}")
    print(f"  val_steps       : {val_steps}")

    # ---------- モデル構築（ゼロから）----------
    print("\n" + "="*55)
    print("モデル構築: MobileNetV3Small + ImageNet重み（ゼロから）")
    print("="*55)
    model, base_model = build_model()
    model.summary(line_length=72)

    history_log     = []
    best_model_path = str(MODEL_DIR / "best_v1_head.keras")

    # ============================================================
    # Step 2: Head のみ学習（backbone 全凍結）
    # ============================================================
    print("\n" + "="*55)
    print("Step 2: Head のみ学習（backbone 全凍結）")
    print("="*55)

    EPOCHS_HEAD   = 80
    schedule_head = make_schedule(
        peak_lr=3e-4,
        epochs=EPOCHS_HEAD,
        steps_per_epoch=steps_per_epoch,
        warmup_epochs=5,
    )
    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=schedule_head),
        loss=tf.keras.losses.CategoricalCrossentropy(label_smoothing=0.05),
        metrics=["accuracy"],
    )
    h_head = model.fit(
        train_ds,
        validation_data=val_ds,
        epochs=EPOCHS_HEAD,
        steps_per_epoch=steps_per_epoch,    # ★ repeat() があるので明示必須
        validation_steps=val_steps,         # ★ 同上
        callbacks=make_callbacks(
            "best_v1_head.keras",
            patience_es=25,
            steps_per_epoch=steps_per_epoch,
            val_steps=val_steps,
        ),
        class_weight=cw,
    )
    history_log.append(("Head", h_head))
    prev_best = min(h_head.history["val_loss"])
    print(f"\n  Head完了: val_loss={prev_best:.4f}  "
          f"val_accuracy={max(h_head.history['val_accuracy']):.4f}")

    # ============================================================
    # Step 3: 段階的 Fine-tuning
    # ★ 各 Stage は前 Stage の最良モデルをロードしてから開始
    # ============================================================
    FT_STAGES = [
        {
            "name":     "FT_A",
            "save":     "best_v1_FT_A.keras",
            "unfreeze": 20,
            "peak_lr":  8e-6,
            "epochs":   60,
            "p_es":     20,
        },
        {
            "name":     "FT_B",
            "save":     "best_v1_FT_B.keras",
            "unfreeze": 40,
            "peak_lr":  3e-6,
            "epochs":   50,
            "p_es":     18,
        },
        {
            "name":     "FT_C",
            "save":     "best_v1_FT_C.keras",
            "unfreeze": None,   # 全層
            "peak_lr":  1e-6,
            "epochs":   40,
            "p_es":     15,
        },
    ]

    for cfg in FT_STAGES:
        print("\n" + "="*55)
        print(f"Step 3 {cfg['name']}: unfreeze={cfg['unfreeze']}層  "
              f"peak_lr={cfg['peak_lr']}")
        print("="*55)

        # 前 Stage の最良モデルをロード
        print(f"  ロード: {best_model_path}")
        model, base_model = load_model_recompile(best_model_path)

        set_unfreeze(base_model, cfg["unfreeze"])

        schedule_ft = make_schedule(
            peak_lr=cfg["peak_lr"],
            epochs=cfg["epochs"],
            steps_per_epoch=steps_per_epoch,
            warmup_epochs=3,
        )
        model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=schedule_ft),
            loss=tf.keras.losses.CategoricalCrossentropy(label_smoothing=0.05),
            metrics=["accuracy"],
        )
        h_ft = model.fit(
            train_ds,
            validation_data=val_ds,
            epochs=cfg["epochs"],
            steps_per_epoch=steps_per_epoch,   # ★ 明示
            validation_steps=val_steps,        # ★ 明示
            callbacks=make_callbacks(
                cfg["save"],
                patience_es=cfg["p_es"],
                steps_per_epoch=steps_per_epoch,
                val_steps=val_steps,
            ),
            class_weight=cw,
        )
        history_log.append((cfg["name"], h_ft))

        stage_best = min(h_ft.history["val_loss"])
        stage_acc  = max(h_ft.history["val_accuracy"])
        print(f"\n  {cfg['name']}完了: val_loss={stage_best:.4f}  "
              f"val_accuracy={stage_acc:.4f}")

        # 崩壊検知（前 Stage より 40% 悪化）
        if stage_best > prev_best * 1.4:
            print(f"\n[!] 崩壊検知: val_loss={stage_best:.4f} > "
                  f"prev_best×1.4={prev_best * 1.4:.4f}")
            print(f"    最良モデルは {best_model_path} のまま維持します。")
            print("    以降の Stage をスキップします。")
            break

        # 改善した場合のみ best_model_path を更新
        if stage_best < prev_best:
            prev_best = stage_best
        best_model_path = str(MODEL_DIR / cfg["save"])
        print(f"  → 次 Stage は {cfg['save']} から開始")

    # ---------- 最終モデル保存 ----------
    print(f"\n最終最良モデル: {best_model_path}")
    final_model, _ = load_model_recompile(best_model_path)
    final_model.save(MODEL_DIR / "final_v1.keras")
    print("最終モデル保存 → models/final_v1.keras")

    # ---------- Step 4: 過学習診断 ----------
    diagnose_overfitting(history_log)

    # ---------- Step 5: Confusion Matrix ----------
    print_confusion_matrix(final_model, val_ds_eval)

    model = tf.keras.models.load_model(
        str(MODEL_DIR / "final_v1.keras"),
        compile=False
    )
    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-5),
        loss=tf.keras.losses.CategoricalCrossentropy(label_smoothing=0.05),
        metrics=["accuracy"]
    )

    # ---------- 最終評価（repeat なし ds で正確に計測）----------
    loss_f, acc_f = final_model.evaluate(val_ds_eval, verbose=1)
    print("\n" + "="*55)
    print("最終結果サマリ")
    print("="*55)
    print(f"  val_loss={loss_f:.4f}  val_accuracy={acc_f:.4f}")
    print(f"""
次のアクション：
  評価                     : python test_model.py --model models/final_v1.keras
  unknown 重みを下げて再実行 : python train_v1.py --unk_weight {max(0.3, args.unk_weight - 0.1):.1f}
  unknown 重みを上げて再実行 : python train_v1.py --unk_weight {min(0.8, args.unk_weight + 0.1):.1f}
""")
    print("train_v1 finished")


if __name__ == "__main__":
    main()