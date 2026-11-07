# Code

TwiceHopeのロボット開発に使用するプログラムをまとめています。Raspberry Pi 5上の`RobCup_V3`を基に、探索、走行制御、センサー処理、画像認識、表示系を用途ごとに整理しています。

## 主なディレクトリ

| ディレクトリ | 内容 |
| --- | --- |
| [`sts3215/`](sts3215/) | 走行制御、経路探索、マッピング |
| [`RoboCup_Maze_World/`](RoboCup_Maze_World/) | センサー・モーター・マップなどのモジュール |
| [`Display_Process/`](Display_Process/) | LCD表示、画面素材、ログ |
| [`Lidar_viwer/`](Lidar_viwer/) | LiDARの計測・可視化 |
| [`MobileNetV3_1/`](MobileNetV3_1/) | カメラ画像の収集、学習、推論 |
| [`src/`](src/) | サーボ制御や走行処理の試作 |
| [`sensors_test/`](sensors_test/)・[`imu_test/`](imu_test/) | センサー・姿勢計測の試験 |
| [`test/`](test/)・[`test_extra/`](test_extra/) | 走行・統合テスト |
| [`camera_test/`](camera_test/)・[`control/`](control/) | カメラ・ボタンの動作確認 |

### ソースコード

Raspberry Pi 5で開発したプログラムを、実機のフォルダー構成に沿って登録しています。仮想環境、実行ログ、キャッシュ、学習済みモデルの本体は含めていません。

フォルダーの詳細は[ファイル構成](../docs/raspi5-file-tree.md)を参照してください。