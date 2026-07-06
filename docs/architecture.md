# 制御構造・アーキテクチャ

## 概要

XIAO ESP32S3を **2個** 使用し、SPIで接続する。GPIOがXIAO ESP32S3 1枚では足りない（センサーI2C×3・GPS UART・ニクロム線×2・モーター用SPIをすべて1枚に収められない）ため、役割ごとに基板を分けた。

```
             SPI（XIAO1がマスター）
XIAO1 ─────────────────────────────► XIAO2
（センサー・GNSS・WiFi）        （PID制御・誘導・モーター駆動）
```

| | XIAO1（マスター） | XIAO2（スレーブ） |
|---|---|---|
| 役割 | センサー読み取り・GPS・WiFi通信 | 誘導フュージョン・PID制御・モーター駆動 |
| 実装方式 | デュアルコア＋FreeRTOSタスク（`loop()`は使わない） | シンプルな`setup()`/`loop()` |
| 主なlib | Sensor, GPS, Radio, SpiLinkMaster | PID, Actuator, SpiLinkSlave |
| PlatformIO env | `xiao1` | `xiao2` |

SPI通信の詳細（フレームフォーマット・ピン）は末尾の「[XIAO1 ⇔ XIAO2 通信（SPI）](#xiao1--xiao2-通信spi)」を参照。

---

## XIAO1 タスク構成

```
Core 0（WiFi/BT系）              Core 1（リアルタイム系）
────────────────────             ──────────────────────────
taskGPS     priority 3           taskSensor   priority 5  ← 50Hz 最優先
taskWifi    priority 2           taskSpiLink  priority 4  ← XIAO2との送受信
```

| タスク | 周期 | 役割 |
|---|---|---|
| taskSensor | 50Hz | IMU・気圧・地磁気を読み、姿勢フィルタを回す |
| taskSpiLink | 20Hz | センサー値・手動操作コマンドをXIAO2へ送信し、XIAO2の実モータ出力を受信してSharedへ書き戻す |
| taskGPS | イベント駆動 | NMEAをバイト単位でパース |
| taskWifi | 20Hz | `Radio::setData()`でフレーム更新＋`Radio::poll()`でHTTPリクエスト処理、地上局からの手動操作コマンドをSharedへ書き込む |

TODO: `taskStateMachine`（ミッションステート遷移）は未実装。現状`Shared.state`は`STANDBY`のまま固定される。

---

## タスク間データ共有（XIAO1内）

XIAO1内の全タスクが `Shared` 構造体 1つを参照する。mutex で排他制御。XIAO2とはこの構造体を共有しておらず、taskSpiLinkがSPI経由で値をやり取りする。

```
include/shared.h
┌───────────────────────────────────────────────┐
│ struct Shared {                               │
│   SemaphoreHandle_t mutex;                    │
│   SensorData latest;          ← 最新センサー値 │
│   MissionState state;         ← 現在のステート │
│   int16_t manualMotorLeft;    ← WiFi手動操作値 │
│   int16_t manualMotorRight;                   │
│ };                                             │
└───────────────────────────────────────────────┘
```

- **書き込み**: taskSensor（latest）、taskGPS（lat/lon）、taskWifi（manualMotorLeft/Right）、taskSpiLink（motorOutputLeft/Right ← XIAO2からの応答）
- **読み取り**: taskWifi（GET /data 配信用）、taskSpiLink（XIAO2への送信用）

---

## ライブラリ構成（lib/）

タスク／`loop()`はlibの薄いラッパーとして機能し、ロジックはすべてlib側に閉じ込める。

```
src/xiao1/tasks/task_xxx.h                    src/xiao2/main.cpp
    │  lib を呼ぶだけ                              │  lib を呼ぶだけ
    ▼                                              ▼
lib/Sensor/          BMP280（気圧）・MPU6050（6軸）・BMM350（地磁気）ドライバ＋姿勢フィルタ   ─ XIAO1
lib/GPS/             TinyGPSPlus をラップしたGPS読み取りクラス（GY-GPSV2-NEO6M）＋方位・距離計算 ─ XIAO1
lib/Radio/           WiFi SoftAP 立ち上げ・HTTP GET でバイナリフレーム配信（PULL方式）        ─ XIAO1
lib/SpiLinkMaster/   XIAO2への送信＋応答受信（全二重SPI、標準SPIライブラリを使用）           ─ XIAO1
lib/SpiLinkSlave/    XIAO1からのトランザクション受信（hideakitai/ESP32SPISlaveを使用）      ─ XIAO2
lib/PID/             汎用PIDコントローラ                                                  ─ XIAO2
lib/Actuator/        モーター（TB6612FNG、2ピン/モーター方式で左右独立駆動）・パラシュート・分離・ブザー・LED ─ XIAO2
lib/StateMachine/    ミッションステート遷移ロジック（未統合。TODO参照）
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

このコマンドはXIAO1のtaskWifiが`Shared.manualMotorLeft/Right`へ書き込み、taskSpiLinkがSPI経由でXIAO2へ転送する（実際にモーターを駆動するのはXIAO2）。手動操作と自律PID（XIAO2側）のどちらを優先するかの調停（`SpiFrameToXiao2::manual_override`）は現状未実装で、常に自律側が優先される。詳細は後述の「XIAO1 ⇔ XIAO2 通信（SPI）」を参照。

なお `tests/test_navigation` はGPS方位とBMM350ヘディングの誤差をPIDで補正し左右モーターへ反映する自律航行の統合確認だが、これはRadio・SPIのどちらも経由しない（Actuatorを直接駆動するスタンドアロンテスト）。XIAO2側の誘導ロジック（`computeAutonomousMotor()`）を実装する際の参考実装として位置づける。

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

---

## XIAO1 ⇔ XIAO2 通信（SPI）

XIAO1がマスター、XIAO2がスレーブ。全二重トランザクション1回で双方向のデータを同時にやり取りする（XIAO2からの応答は1サイクル前の計算結果になる）。

```
XIAO1（マスター）                              XIAO2（スレーブ）
  taskSpiLink（20Hz）                            loop()
      │  SpiLinkMaster::transfer(out) ──────────►│ SpiLinkSlave::poll(in)
      │  ＝ SPI.transferBytes()（全二重）          │ ＝ ESP32SPISlaveのqueue/trigger方式
      │◄────────────────────────────────────────│ SpiLinkSlave::setResponse(out)
      ▼
  Sharedへ書き戻し（motorOutputLeft/Right）
```

契約は `include/spi_protocol.h` にある。変更する場合は `lib/SpiLinkMaster` と `lib/SpiLinkSlave` 両方への影響を確認すること。

**XIAO1 → XIAO2（`SpiFrameToXiao2`）**

| フィールド | 型 | 備考 |
|---|---|---|
| timestamp_ms | uint32 | 起動からのms |
| alt / roll / pitch / yaw | float32 | XIAO1のSensorでフュージョン済みの姿勢・高度 |
| lat / lon | float32 | GPS座標（doubleから縮小） |
| mission_state | uint8 | `MissionState`のenum値 |
| manual_motor_left / right | int16 | 地上局からのWiFi手動操作値（-255〜255） |
| manual_override | uint8 | 1=手動優先、0=自律PID優先。**TODO: 調停ロジック未実装、常に0** |

**XIAO2 → XIAO1（`SpiFrameFromXiao2`）**

| フィールド | 型 | 備考 |
|---|---|---|
| motor_output_left / right | int16 | XIAO2が実際にモーターへ出力した値（手動 or PID後）。XIAO1がWiFiテレメトリとして配信する |

### XIAO2側の処理（誘導・PID）

`src/xiao2/main.cpp` の `computeAutonomousMotor()` が、受信した`yaw`と目標方位との誤差をPIDで補正し、左右モーターの差動出力に変換する（`tests/test_navigation` と同じ考え方）。現状は骨組みのみで、以下がTODO：

- 目標座標（`targetLat`/`targetLon`）の与え方（現状は固定値0,0）
- 方位誤差の計算（bearing計算・-180〜180度への正規化）
- PIDゲインの実機調整
- XIAO1からの通信が一定時間途絶えた場合のフェイルセイフ（モータ停止）

### SPIピン・ビルド上の注意

- ピン番号は `include/spi_protocol.h` の `SpiPins` 名前空間で定義（現状は仮値、回路図の実ピンに合わせて要修正）。
- XIAO2は `hideakitai/ESP32SPISlave` ライブラリに依存する（`platformio.ini` の `env:xiao2` にのみ追加）。マスター用（`lib/SpiLinkMaster`）とスレーブ用（`lib/SpiLinkSlave`）を別々のlibフォルダに分けているのは、PlatformIOのライブラリ依存解決がフォルダ単位でソースをコンパイルするため、同じフォルダに同居させるとXIAO1のビルドにもXIAO2専用ライブラリへの依存が混入してしまうことを避けるため。
