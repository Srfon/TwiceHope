# 直線検出・走行判断の接続方法

`ms200_viewer/navigation.py` は次を実装しています。

- 点群を専用の二値画像へ変換（表示用グリッドは混入させない）
- 3×3カーネルによるモルフォロジー収縮
- `HoughLinesP` による直線抽出
- 前方直線まで50 mm、またはエンコーダー積算300 mmで停止
- 停止後0.5秒待って前・左・右を観測
- 開いていれば0、閉じていれば1として隣接座標を `(x, y, n)` で保存
- 経路選択は前、左、右の順
- 平行線に対して3度以上傾いていれば旋回指令を出す

モーターを直接操作するコードは意図的に含めていません。制御ループでは概ね次の形で
使用します。

```python
detector = LineDetector()
navigator = Navigator()

perception = detector.detect(visible_points)
pose = Pose(
    x_mm=encoder_pose_x,
    y_mm=encoder_pose_y,
    heading_deg=imu_or_odometry_heading,
    travelled_mm=encoder_total_distance,
)
command = navigator.update(perception, pose)

# commandを実機固有のモータードライバーへ渡す
# STOP / FORWARD / TURN_LEFT / TURN_RIGHT
```

30 cm判定には左右ホイールエンコーダー等の積算距離が必要です。LiDAR画像のフレーム数を
距離として使用してはいけません。また、角度補正と90度旋回にはIMUまたはエンコーダーから
得た `heading_deg` が必要です。最初は車輪を浮かせ、低速・非常停止可能な状態で指令の向きと
停止動作を確認してください。

## 実機層

添付されたSTS3215の4輪コードとTCA9548A経由のVL53L1Xコードは
`ms200_viewer/robot_hardware.py` に整理しています。`SafeMotionBridge` は前進指令時に
VL53L1Xが50 mm以下を示した場合、または0.5秒以上測距を更新できない場合に停止します。

```python
drive = FourWheelDrive()
front_sensor = FrontDistanceSensor()
drive.initialize()
front_sensor.start()
motors = SafeMotionBridge(drive, front_sensor)

actual_command = motors.apply(command)
```

例外発生時を含め、終了処理では必ず次を実行します。

```python
motors.emergency_stop()
front_sensor.stop()
drive.shutdown()
```

まだ30 cm走行判定は実機層へ接続していません。STS3215の現在位置レジスターを読むAPIと
車輪径が分からない状態で、時間だけから30 cmを推定すると停止距離が電圧・床面・負荷で
変化するためです。
