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

### コードの公開状況

現在の`Code/`は、実機にあるファイル名と階層を再現した**空のひな形**です。Pythonソース、画像、モデルの実体はまだ反映していません。この状態のファイルを実機の同名ファイルに上書きしないでください。

元の構成は[ファイル一覧](../docs/raspi5-file-tree.md)で確認できます。実装を公開する際は、仮想環境（`.venv`）、キャッシュ、ログ、認証情報を除外します。
