# 学習済みモデル

画像認識に使用するモデルの保存先です。実機の構成にはKeras形式の`final_v1.keras`と、`final_v1.tflite`、`final_v1_int8.tflite`、`final_v1_float32.tflite`があります。

Raspberry Piでの推論を想定し、モデル形式や量子化方式による処理速度・認識精度の違いを検証します。

学習済みモデルの本体は、現在このリポジトリに含めていません。
