# TwiceHope

RoboCup Rescue Maze ロボット開発プロジェクト（2027年大会に向けた開発）

## 概要

**TwiceHope** は RoboCup Rescue Maze（Rescue World Maze）の自律走行ロボットを開発するチームです。迷路探索・地図生成・被災者認識・救助キット投下を扱います。

> このリポジトリは初期構成の段階です。実機で動作する既存コードはまだ登録していません。

## 構成（計画）

- **Raspberry Pi 5**: LiDAR・カメラの処理、マッピング、経路探索、Web/LCD表示
- **LEGO SPIKE Prime**: モーターとセンサーの制御
- **UART**: Raspberry Pi と SPIKE Prime の通信
- **RPLIDAR A1M8**: 周辺の距離測定
- **USBカメラ（左右）**: H/S/U文字と色付き被災者の認識
- **カラーセンサー**: タイル色の判別

## ディレクトリ

```text
TwiceHope/
├── README.md
├── .gitignore
├── Display_Process/
│   ├── Png/           # LCD・Web表示とdisplay_manager5関連
│   └── Log/           # 実行ログ（*.logはGit管理対象外）
├── sts3215/           # robot_mapping5関連
├── spike/             # SPIKE Primeの走行制御
└── docs/              # アーキテクチャ・開発メモ
```

> 上記は今後既存コードを格納するための配置方針です。空のフォルダーを保持するため、各フォルダーに説明用READMEを置きます。

## 現行コードの配置方針

既存環境とパスを合わせるため、次を基本配置とします。

| 役割 | 配置先の例 |
| --- | --- |
| ロボットの探索・マッピング | `sts3215/robot_mapping5.py` |
| LCD表示管理 | `Display_Process/Png/display_manager5.py` |
| ログ管理 | `Display_Process/Log/Logger5.py` |
| SPIKE Prime制御 | `spike/` |

※ ファイル名・拡張子は実コードを受け取ってから確認します。**上記ファイルが既にアップロードされているという意味ではありません。**

## 開発上の注意

- パスワード、Wi-Fi認証情報、トークン、秘密鍵をコミットしない
- `.env`、実行ログ、Pythonキャッシュ、一時ファイルはGit管理から除外する
- 実機制御コードは動作確認しながら段階的に導入する

詳しい構成は [docs/architecture.md](docs/architecture.md) を参照してください。
