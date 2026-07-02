# 制御構造・アーキテクチャ

## 概要

ESP32-S3（XIAO ESP32S3）のデュアルコア＋FreeRTOSを活用し、センサー読み取り・状態遷移・WiFi通信を並列で実行する。`loop()`は使わず、すべてをFreeRTOSタスクに委ねる構成。

---

## タスク構成

```
Core 0（WiFi/BT系）              Core 1（リアルタイム系）
────────────────────             ──────────────────────────
taskGPS     priority 3           taskSensor       priority 5  ← 50Hz 最優先
taskWifi    priority 2           taskStateMachine priority 4
                                 （Actuator はSMから直接呼ぶ）
```

| タスク | 周期 | 役割 |
|---|---|---|
| taskSensor | 50Hz | IMU・気圧・地磁気を読み、姿勢フィルタを回す |
| taskStateMachine | 10Hz | ステート判定・アクチュエータ制御 |
| taskGPS | イベント駆動 | NMEAをバイト単位でパース |
| taskWifi | 20Hz | `Radio::setData()`でフレーム更新＋`Radio::poll()`でHTTPリクエスト処理 |

---

## タスク間データ共有

全タスクが `Shared` 構造体 1つを参照する。mutex で排他制御。

```
include/shared.h
┌─────────────────────────────────────────┐
│ struct Shared {                         │
│   SemaphoreHandle_t mutex;              │
│   SensorData latest;   ← 最新センサー値 │
│   MissionState state;  ← 現在のステート │
│ };                                      │
└─────────────────────────────────────────┘
```

- **書き込み**: taskSensor（latest）、taskGPS（lat/lon）、taskStateMachine（state）
- **読み取り**: taskWifi（GET /data 配信用）、taskStateMachine（判定用）

---

## ライブラリ構成（lib/）

タスクはlibの薄いラッパーとして機能し、ロジックはすべてlib側に閉じ込める。

```
src/tasks/task_xxx.h
    │  lib を呼ぶだけ
    ▼
lib/Sensor/          BMP280（気圧）・MPU6050（6軸）・BMM350（地磁気）ドライバ＋姿勢フィルタ
lib/GPS/             TinyGPSPlus をラップしたGPS読み取りクラス（GY-GPSV2-NEO6M）＋方位・距離計算
lib/PID/             汎用PIDコントローラ
lib/Radio/           WiFi SoftAP 立ち上げ・HTTP GET でバイナリフレーム配信（PULL方式）
lib/Actuator/        モーター（TB6612FNG、2ピン/モーター方式で左右独立駆動）・パラシュート・分離・ブザー・LED
lib/StateMachine/    ミッションステート遷移ロジック
```

---

## ミッションステート遷移

```
              高度上昇検知
  STANDBY ─────────────────► ASCENDING
                                  │
                         高度減少 or 衝撃検知
                                  │
                                  ▼
                             DESCENDING
                                  │
                           着地高度到達
                                  │
                                  ▼
                            SEPARATING  ── パラシュート展開・分離
                                  │
                                  ▼
                              RUNNING   ── 地上走行・ミッション
                                  │
                            タイムアウト or ゴール到達
                                  │
                                  ▼
                             GOAL/MISSING ── ブザー・LED点滅（回収支援）
```

各遷移のトリガー：

| 遷移 | トリガー |
|---|---|
| STANDBY → ASCENDING | 気圧高度が閾値を超える |
| ASCENDING → DESCENDING | 高度減少 |
| DESCENDING → SEPARATING | 低高度到達 |
| SEPARATING → RUNNING | 分離処理完了後即時 |
| RUNNING → GOAL | タイムアウト |

---

## 無線通信

地上PCとはWiFi SoftAPで常時接続し、PULL方式（地上PC側がHTTP GETで定期的に取りに行く）でテレメトリを配信する。SDカードは使用しない。

機体→PCへのPUSH（WebSocketブロードキャストやPOST）は、PC側ファイアウォールに着信をブロックされる環境があり信頼できないため採用しない。地上PCからのアウトバウンドGETは許可されやすいため、PULL方式に統一している。

```
ESP32-S3（SoftAP: CanSat-AP）
    │  GET http://192.168.4.1/data
    │  33バイト バイナリフレーム（ポーリング間隔 100〜150ms 目安）
    ▼
地上PC（ブラウザ or Pythonスクリプト）
    └─ データ記録・リアルタイム表示
```

`/data` は `Radio::setData()` が更新する最新スナップショットを、リクエストごとに読み直して返す（`Radio::poll()` を呼ぶたびに保留中のHTTPリクエストを処理する）。ブラウザ用の簡易ダッシュボード（`GET /`）も同じ `/data` を`fetch()`でポーリングする。

地上局からのモーター手動制御は `GET /motor?left=-255〜255&right=-255〜255` で受け付ける（PCから機体へのアウトバウンド方向なので、`/data`のPULLと同じく信頼できる方向）。`Radio::getMotorCommandLeft()`/`getMotorCommandRight()` は、`MOTOR_COMMAND_TIMEOUT_MS`（1000ms）以上新しいコマンドが来ていなければ自動的に0を返すフェイルセイフを持つ。地上局側（`ground/receiver.py`）は手動制御が有効な間、250ms間隔でコマンドを送り続けることでこのタイムアウトより十分短い周期を維持する。

なお `tests/test_navigation` はGPS方位とBMM350ヘディングの誤差をPIDで補正し左右モーターへ反映する自律航行の統合確認だが、これはRadioを経由しない（Actuatorを直接駆動するスタンドアロンテスト）。本番の`taskStateMachine`にこのロジックを統合する際は、Radioの手動制御コマンドと自律制御のどちらを優先するかの調停が今後必要になる。

受信バイナリフレームフォーマット（リトルエンディアン、33バイト。`lib/Radio/Radio.h` と `ground/receiver.py` で共有する契約）：

| Offset | Size | 型 | フィールド | 備考 |
|---|---|---|---|---|
| 0 | 4 | uint32 | timestamp_ms | 起動からのms |
| 4 | 4 | float32 | alt | 高度 [m] |
| 8 | 4 | float32 | roll | ロール角 [deg] |
| 12 | 4 | float32 | pitch | ピッチ角 [deg] |
| 16 | 4 | float32 | yaw | ヨー角 [deg] |
| 20 | 4 | float32 | lat | 緯度（doubleから縮小） |
| 24 | 4 | float32 | lon | 経度（doubleから縮小） |
| 28 | 1 | uint8 | mission_state | `MissionState`のenum値 |
| 29 | 2 | int16 | motor_output_left | `Actuator::setMotorLeft()`相当値（-255〜255） |
| 31 | 2 | int16 | motor_output_right | `Actuator::setMotorRight()`相当値（-255〜255） |
