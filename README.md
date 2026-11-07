# TwiceHope

**RoboCup Rescue Maze / Rescue World Maze** の自律走行ロボット開発リポジトリ（2027年大会に向けた開発）。

## Raspberry Pi 5 実機の開発ディレクトリ

実機パス: `/home/trin1/RobCup_V3/`

提供されたファイル一覧を確認し、**211個のファイルパス**を記録しました。**完全なフォルダー・ファイル一覧は [docs/raspi5-file-tree.md](docs/raspi5-file-tree.md)** にあります。

主な実機フォルダー:

- `__pycache__/`
- `.venv/`
- `.vscode/`
- `camera_test/`
- `control/`
- `Display_Process/`
- `imu_test/`
- `Lidar_viwer/`
- `MobileNetV3_1/`
- `RoboCup_Maze_World/`
- `sensors_test/`
- `src/`
- `sts3215/`
- `test/`
- `test_extra/`

実機内で確認できた主要ファイル:

| 機能・用途 | 実機上の相対パス |
| --- | --- |
| ロボット探索・マッピング | `sts3215/robot_mapping5.py` |
| 表示管理 | `Display_Process/display_manager5.py` |
| ログ処理 | `Display_Process/Log/Logger5.py` |
| 表示PNG | `Display_Process/Png/` |
| LiDAR検証 | `Lidar_viwer/` |
| 機械学習・カメラ認識検証 | `MobileNetV3_1/` |
| ロボット各機能の整理 | `RoboCup_Maze_World/` |
| Python依存関係 | `requirements.txt` |
| ナビゲーション資料 | `NAVIGATION.md` |

## 主要構成

- **Raspberry Pi 5**: LiDAR、カメラ、地図・経路探索、Web/LCD表示
- **LEGO SPIKE Prime**: 走行/投下機構・一部センサーの制御（接続プロトコルは実機コードと照合予定）
- **RPLIDAR A1M8**: 壁や障害物までの距離測定
- **USBカメラ**: H/S/U文字・被災者色の認識

## リポジトリの状態

現段階では、**実機のディレクトリ一覧と構成ドキュメントの登録まで完了**しています。Raspberry Pi内のPythonソース、画像、学習済みモデルなどの実体はまだ同期していません。既存GitHub上の `spike/` は今後の配置用メモであり、提供された実機一覧には存在しません。

- [実機の全ファイル構成](docs/raspi5-file-tree.md)
- [アーキテクチャ・開発課題](docs/architecture.md)

## 安全な運用

秘密鍵・パスワード・Wi-Fi接続情報・`.env`・仮想環境・ログをコミットしないでください。モーター制御に関わる変更は実機の安全対策の上で検証してください。
