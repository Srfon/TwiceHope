# Raspberry Pi 5 実機フォルダー構成 — RobCup_V3

- 対象: `/home/trin1/RobCup_V3/`
- 記録日: 2026-11-07
- 情報源: ユーザーが提供したファイルパス一覧（**211ファイル**）
- **注意**: これはファイルの**存在位置を記録した一覧**であり、実ファイルの中身は本リポジトリにまだ登録されていません。提供された一覧にないファイルや空ディレクトリは確認できません。
- `.venv/`、`__pycache__/`、ログなどは実機に存在しますが、通常はGit管理対象外です。
- 実際のスペル（例: `Lidar_viwer`）・大文字小文字・数字・ファイル名を元の一覧どおり維持しています。

## 全ファイル構成

```text
RobCup_V3/
├── __pycache__/
│   ├── IMU_Caluc.cpython-311.pyc
│   └── VL53L1X.cpython-311.pyc
├── .venv/
│   ├── bin/
│   │   ├── __pycache__/
│   │   │   ├── ftconf.cpython-311.pyc
│   │   │   ├── ftdi_urls.cpython-311.pyc
│   │   │   ├── i2cscan.cpython-311.pyc
│   │   │   └── pyterm.cpython-311.pyc
│   │   ├── activate
│   │   ├── activate.csh
│   │   ├── activate.fish
│   │   ├── Activate.ps1
│   │   ├── cbor2
│   │   ├── dotenv
│   │   ├── f2py
│   │   ├── fastapi
│   │   ├── fonttools
│   │   ├── ftconf.py
│   │   ├── ftdi_urls.py
│   │   ├── i2cscan.py
│   │   ├── idna
│   │   ├── markdown-it
│   │   ├── normalizer
│   │   ├── numpy-config
│   │   ├── pinout
│   │   ├── pintest
│   │   ├── pip
│   │   ├── pip3
│   │   ├── pip3.11
│   │   ├── pyftmerge
│   │   ├── pyftsubset
│   │   ├── pygmentize
│   │   ├── pyserial-miniterm
│   │   ├── pyserial-ports
│   │   ├── pyterm.py
│   │   ├── saved_model_cli
│   │   ├── tf_upgrade_v2
│   │   ├── tflite_convert
│   │   ├── toco
│   │   ├── ttx
│   │   ├── uvicorn
│   │   ├── watchfiles
│   │   ├── websockets
│   │   └── wheel
│   ├── .gitignore
│   └── pyvenv.cfg
├── .vscode/
│   ├── launch.json
│   └── settings.json
├── camera_test/
│   └── get_img.py
├── control/
│   ├── button_display.py
│   └── button_test.py
├── Display_Process/
│   ├── Log/
│   │   ├── __pycache__/
│   │   │   └── Logger5.cpython-311.pyc
│   │   ├── Logger.py
│   │   ├── Logger5.py
│   │   └── system.log
│   ├── Png/
│   │   ├── Base.png
│   │   ├── goal.png
│   │   ├── omega.png
│   │   ├── phi.png
│   │   ├── psi.png
│   │   ├── rb_end.png
│   │   ├── rb_retry.png
│   │   └── rb_start.png
│   ├── display_manager.py
│   ├── display_manager2.py
│   ├── display_manager3.py
│   ├── display_manager5.py
│   └── README.md
├── imu_test/
│   ├── a.py
│   ├── imu_binary.py
│   ├── imu_binary2.py
│   ├── imu_binary3.py
│   ├── imu_old.py
│   ├── imu_raw.py
│   ├── imu_raw2.py
│   ├── imu_raw3.py
│   ├── imu_test1.py
│   ├── imu_test2_comp.py
│   ├── imu_test3_comp.py
│   ├── imu_ver1.py
│   ├── imu.py
│   └── log_yaw.txt
├── Lidar_viwer/
│   ├── lidar_imu_display.py
│   ├── test_lidar_imu_display2.py
│   ├── test_lidar_imu2.py
│   ├── test_lidar_imu3.py
│   ├── test_lidar_imu4.py
│   ├── test_lidar_serial.py
│   ├── test_lidar_web.py
│   ├── test_lidar_web2.py
│   ├── test_lidar_web3.py
│   ├── test_lidar_web4.py
│   └── test_lidar.py
├── MobileNetV3_1/
│   ├── models/
│   │   ├── final_v1_float32.tflite
│   │   ├── final_v1_int8.tflite
│   │   ├── final_v1.keras
│   │   └── final_v1.tflite
│   ├── Collect_Data.py
│   ├── convert_into_tflite_int8.py
│   ├── RealTime_test_from_SSH.py
│   ├── RealTime_test_tflite.py
│   ├── RealTime_test.py
│   └── train_V3_new.py
├── RoboCup_Maze_World/
│   ├── Map/
│   │   └── Map.py
│   ├── Motor/
│   │   ├── __pycache__/
│   │   │   └── sts3215.cpython-311.pyc
│   │   ├── Motor.py
│   │   └── sts3215.py
│   ├── Sensors/
│   │   ├── IMU/
│   │   │   └── IMU.py
│   │   ├── LiDAR/
│   │   │   └── LiDAR.py
│   │   ├── Temp/
│   │   │   └── Temp.py
│   │   ├── ToF/
│   │   │   └── ToF.py
│   │   └── Under_Camera/
│   │       └── Under_Camera.py
│   ├── State/
│   │   └── State.py
│   └── README.md
├── sensors_test/
│   ├── display_off.py
│   ├── display_test.py
│   ├── game.py
│   ├── gyro_viewer.py
│   ├── IMU_Caluc.py
│   ├── IMU_Tester.py
│   ├── Receive_data.py
│   ├── sensor.py
│   ├── TCA9548A.py
│   ├── TCA9548A2.py
│   ├── TCA9548A3.py
│   ├── temp_sensor.py
│   └── test2.py
├── src/
│   ├── __pycache__/
│   │   ├── lidar_drive.cpython-311.pyc
│   │   ├── robot.cpython-311.pyc
│   │   └── Servo_cli3.cpython-311.pyc
│   ├── servo/
│   │   ├── __pycache__/
│   │   │   ├── Servo_cli.cpython-311.pyc
│   │   │   ├── sts3032_3.cpython-311.pyc
│   │   │   ├── sts3032_4.cpython-311.pyc
│   │   │   └── sts3032.cpython-311.pyc
│   │   ├── protocol.py
│   │   ├── Servo_cli_spec.md
│   │   ├── Servo_cli.py
│   │   ├── sts3032_3.py
│   │   ├── sts3032_4.py
│   │   ├── sts3032_5.py
│   │   └── sts3032.py
│   ├── lidar_drive.py
│   ├── robot.py
│   ├── Servo_APIs_pre.py
│   ├── Servo_APIs.py
│   ├── Servo_APIs2.py
│   ├── Servo_cli3.py
│   ├── Servo.cli.py
│   ├── Servo.cli2.py
│   ├── test_servo.py
│   ├── test_servo2.py
│   ├── test_servo3.py
│   ├── test_servo4.py
│   └── test1.py
├── sts3215/
│   ├── __pycache__/
│   │   ├── robot_moving2.cpython-311.pyc
│   │   └── sts3215.cpython-311.pyc
│   ├── ms200_check.py
│   ├── res_ids.py
│   ├── robot_driving.py
│   ├── robot_hardware_test.py
│   ├── robot_mapping.py
│   ├── robot_mapping2_overlay.py
│   ├── robot_mapping5_2.py
│   ├── robot_mapping5_3.py
│   ├── robot_mapping5.py
│   ├── robot_mapping5.py.bak
│   ├── robot_mapping6.py
│   ├── robot_mapping7.py
│   ├── robot_moving.py
│   ├── robot_moving2.py
│   ├── sts3215.py
│   ├── test_mode_speed_torque.py
│   ├── test_move.py
│   ├── test_on_terminal.py
│   ├── test_status.py
│   ├── test_sts3215.py
│   ├── test1.py
│   └── test2.py
├── test/
│   ├── __pycache__/
│   │   ├── navigation.cpython-311.pyc
│   │   ├── robot_hardware.cpython-311.pyc
│   │   ├── side_sensor_test.cpython-311.pyc
│   │   └── sts3215.cpython-311.pyc
│   ├── distance_30cm_test.py
│   ├── front_stop_test.py
│   ├── lidar_drive_test.py
│   ├── main.py
│   ├── motor_forward_test.py
│   ├── navigation.py
│   ├── protocol.py
│   ├── render.py
│   ├── res_ids.py
│   ├── robot_hardware.py
│   ├── side_guidance_test.py
│   ├── side_sensor_test.py
│   ├── sts3215.py
│   └── viewer.py
├── test_extra/
│   ├── test_all5.py
│   ├── test.jpg
│   ├── TestAll.py
│   ├── TestAll2.py
│   ├── TestAll3.py
│   ├── TestAll4_browser copy 2.py
│   ├── TestAll4_browser copy.py
│   └── TestAll4_browser.py
├── NAVIGATION.md
├── requirements.txt
├── robot_button_start.log
├── show_ch_tca.py
├── sts3215_4wd_debug.log
├── sts3215_debug.log
├── tile_map.drive.jsonl
├── tile_map.json
└── viewer.py
```

## GitHubへソースを登録する際の注意

- 登録予定: `Display_Process/`、`Lidar_viwer/`、`RoboCup_Maze_World/`、`MobileNetV3_1/`、`sts3215/`、`src/`、`test/` などのソース・必要な文書
- 除外推奨: `.venv/`、`__pycache__/`、`*.pyc`、`*.log`、秘密鍵、認証情報
- `MobileNetV3_1/models/` の学習済みモデルはファイルサイズやライセンスを確認してからGit LFS等の管理方法を決定する
- `tile_map.json`、`tile_map.drive.jsonl` 等の生成データは内容と共有可否を確認する
