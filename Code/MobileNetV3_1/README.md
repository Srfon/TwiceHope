# 画像認識 — MobileNetV3

カメラ画像を使った認識処理の学習・推論関連ファイルをまとめています。RoboCup Rescue Mazeで必要となる被災者の検出に向け、データ収集から実時間での推論までを試します。

| ファイル | 用途 |
| --- | --- |
| `Collect_Data.py` | 学習用画像の収集 |
| `train_V3_new.py` | モデルの学習 |
| `convert_into_tflite_int8.py` | TFLite形式への変換 |
| `RealTime_test.py` | カメラ入力での認識テスト |
| `RealTime_test_tflite.py` | TFLiteでの推論テスト |
| [`models/`](models/) | 学習済みモデルの保管場所 |

Raspberry Pi上で動かすことを考え、認識精度と処理速度のバランスも検証対象としています。
