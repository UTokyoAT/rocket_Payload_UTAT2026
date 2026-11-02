# rocket_Payload_UTAT2026

輸送機班2026年度の新人研修期間中の上級生での活動。CanSatを搭載したペットボトルロケットを開発するためのリポジトリ

---

## ドキュメント

- [環境構築](docs/setup.md)（未作成）
- [書き込み方法](docs/flashing.md)（未作成）
- [制御構造・アーキテクチャ](docs/architecture.md)（未作成）
- [地上局セットアップ](docs/ground-station.md)（未作成）
- [JLCPCB 発注手順](docs/pcb-ordering.md)（未作成）

---

## ディレクトリ構造

Seeed XIAO ESP32-S3 を1枚使う構成。ミッション本体は `env:mission`（`tests/test_mission/main.cpp`）。

```
rocket_Payload_UTAT2026/
│
├── platformio.ini           # ビルド・ボード・ライブラリ設定（env:mission / env:test-* ）
│
├── include/
│   └── secrets.example.h    # Wi-Fi・MQTT接続設定のひな形（secrets.h にコピーして使う）
│
├── src/
│   ├── MotorController/     # 単純な方位PID → 左右PWMの差動
│   ├── Actuator/            # モーター制御（TB6612FNG）
│   ├── Deployer/            # ロケット分離・パラシュート分離（ニクロム線）
│   └── LED/                 # LED制御
│
├── lib/
│   ├── Encoder/             # 左右ホイールエンコーダ（ホール、A/B相の割り込みカウント）
│   ├── Expander/            # I/Oエキスパンダ（MCP23008）
│   ├── GPS/                 # TinyGPSPlus ラッパー
│   ├── KarmanFilter/        # 拡張カルマンフィルタ（EKF）
│   ├── Sensor/              # BMP280・MPU6050 ドライバ
│   └── Telemetry/           # Wi-Fi + MQTT で地上局へ送信
│
├── tests/
│   ├── test_mission/        # ミッション本体（キャリブレーション → EKF起動 → 誘導 → 停止）
│   └── test_*/              # センサー・アクチュエータ単体の動作確認（env:test-*）
│
├── matlab/
│   └── ekf_cansat_simulation/  # EKF・誘導のシミュレーション（符号規約の参照実装）
│
├── ground/
│   └── server/              # 地上局（Mosquitto・Telegraf・InfluxDB・Grafana の docker compose）
│
└── hardware/
    ├── kicad/               # KiCad プロジェクトファイル
    └── jlcpcb/
        ├── gerbers/         # Gerberファイル（発注用ZIP）
        ├── bom/             # Bill of Materials
        └── cpl/             # Component Placement List
```
