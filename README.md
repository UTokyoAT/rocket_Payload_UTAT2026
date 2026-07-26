# rocket_Payload_UTAT2026

輸送機班2026年度の新人研修期間中の上級生での活動。CanSatを搭載したペットボトルロケットを開発するためのリポジトリ

---

## ドキュメント

- [環境構築](docs/setup.md)
- [書き込み方法](docs/flashing.md)
- [制御構造・アーキテクチャ](docs/architecture.md)
- [地上局セットアップ](docs/ground-station.md)
- [JLCPCB 発注手順](docs/pcb-ordering.md)

---

## ディレクトリ構造

XIAO ESP32S3を2枚（XIAO1・XIAO2）使い、SPIで接続する構成。詳細は [architecture.md](docs/architecture.md) を参照。

```
rocket_Payload_UTAT2026/
│
├── platformio.ini           # ビルド・ボード・ライブラリ設定（env:xiao1 / env:xiao2 他）
│
├── include/
│   ├── shared.h             # XIAO1タスク間共有データ（SensorData・MissionState）
│   └── spi_protocol.h       # XIAO1⇔XIAO2間のSPIフレーム定義（SpiFrameToXiao2 他）
│
├── src/
│   ├── xiao1/                       # センサー・GPS・誘導PID計算（SPIマスター）
│   │   ├── main.cpp                 # 起動・タスク生成
│   │   └── tasks/
│   │       ├── task_sensor.h        # IMU・気圧・地磁気読み取り＋姿勢フィルタ（100Hz, Core1）
│   │       ├── task_gps.h           # GPS受信・パース（Core0）
│   │       ├── task_navigation.h    # 誘導PID計算（100Hz, Core1）
│   │       └── task_spi_link.h      # XIAO2への送信（100Hz, Core1）
│   └── xiao2/                       # モータ駆動・WiFiテレメトリ中継（SPIスレーブ）
│       └── main.cpp
│
├── lib/
│   ├── Sensor/              # BMP280・MPU6050・BMM350 ドライバ＋姿勢フィルタ  ─ XIAO1
│   ├── GPS/                 # TinyGPSPlus ラッパー                          ─ XIAO1
│   ├── PID/                 # 汎用PIDコントローラ                          ─ XIAO1
│   ├── SpiLinkMaster/       # XIAO2へのSPI送信（マスター側）                ─ XIAO1
│   ├── SpiLinkSlave/        # XIAO1からのSPI受信（スレーブ側）              ─ XIAO2
│   ├── Radio/               # WiFi SoftAP・HTTP GETでバイナリフレーム配信（PULL方式） ─ XIAO2
│   ├── Actuator/            # モーター・パラシュート・ブザー・LED制御       ─ XIAO2
│   └── StateMachine/        # ミッションステート遷移ロジック（未統合）
│
├── ground/
│   ├── receiver.py          # HTTP GETポーリング受信＋CSVロギング＋Tkinter GUI
│   ├── mock_device.py       # 実機なしでreceiver.pyを確認するためのモック
│   ├── requirements.txt
│   └── logs/                # 保存されたCSVログ（.gitignore推奨）
│
├── hardware/
│   ├── kicad/               # KiCad プロジェクトファイル
│   └── jlcpcb/
│       ├── gerbers/         # Gerberファイル（発注用ZIP）
│       ├── bom/             # Bill of Materials
│       └── cpl/             # Component Placement List
│
└── docs/                    # ドキュメント
```
