# rocket_Payload_UTAT2026

輸送機班2026年度の新人研修期間中の上級生での活動。CanSatを搭載したペットボトルロケットを開発するためのリポジトリ

---

## ドキュメント

- [環境構築](docs/setup.md)
- [書き込み方法](docs/flashing.md)
- [制御構造・アーキテクチャ](docs/architecture.md)
- [JLCPCB 発注手順](docs/pcb-ordering.md)

---

## ディレクトリ構造

```
rocket_Payload_UTAT2026/
│
├── platformio.ini           # ビルド・ボード・ライブラリ設定
│
├── include/
│   └── shared.h             # タスク間共有データ（SensorData・MissionState）
│
├── src/
│   ├── main.cpp             # 起動・ハードウェア初期化・タスク生成
│   └── tasks/
│       ├── task_sensor.h    # センサー読み取りタスク（50Hz, Core1）
│       ├── task_gps.h       # GPS受信・パースタスク（Core0）
│       ├── task_wifi.h      # WebSocket ストリーミングタスク（20Hz, Core0）
│       └── task_statemachine.h  # ステート遷移タスク（10Hz, Core1）
│
├── lib/
│   ├── Sensor/              # BMP388・ICM-42688・QMC5883 ドライバ＋姿勢フィルタ
│   ├── GPS/                 # TinyGPSPlus ラッパー
│   ├── Radio/               # WiFi SoftAP・WebSocket ブロードキャスト
│   ├── Actuator/            # モーター・パラシュート・ブザー・LED制御
│   └── StateMachine/        # ミッションステート遷移ロジック
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
