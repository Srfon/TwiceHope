# 走行・統合テスト

走行制御とセンサー入力、ナビゲーションを組み合わせた動作確認のためのコードです。移動距離や停止位置、壁との距離を利用した制御などを個別に試します。

- `motor_forward_test.py`、`distance_30cm_test.py` — 移動量の確認
- `front_stop_test.py`、`side_sensor_test.py` — 距離センサーを利用した停止・誘導
- `navigation.py`、`robot_hardware.py`、`main.py` — 機体制御とナビゲーションの検証
- `viewer.py`、`render.py` — 状態表示関連

試験条件を分けて、不具合の再現と修正に役立てています。
