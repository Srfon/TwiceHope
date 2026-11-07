# 学習済みモデル

画像認識に使用するモデルの保存先です。実機の構成にはKeras形式の`final_v1.keras`と、`final_v1.tflite`、`final_v1_int8.tflite`、`final_v1_float32.tflite`があります。

Raspberry Piでの推論を想定し、モデル形式や量子化方式による処理速度・認識精度の違いを検証します。

現在GitHubにある同名ファイルは構成確認用の空ファイルで、学習済みモデル本体は含まれていません。
