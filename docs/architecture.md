# 制御構造・アーキテクチャ

## 概要

XIAO ESP32S3を **2個** 使用し、SPIで接続する。GPIOがXIAO ESP32S3 1枚では足りない（センサーI2C×3・GPS UART・ニクロム線×2・モーター用SPIをすべて1枚に収められない）ため、役割ごとに基板を分けた。

```
             SPI（XIAO1がマスター）
XIAO1 ─────────────────────────────► XIAO2
（センサー・GNSS・誘導PID）      （モーター駆動・WiFiテレメトリ中継）
```

| | XIAO1（マスター） | XIAO2（スレーブ） |
|---|---|---|
| 役割 | センサー読み取り・GPS・誘導フュージョン・PID制御 | モーター駆動・WiFi通信（テレメトリ配信・手動操作受信） |
| 実装方式 | デュアルコア＋FreeRTOSタスク（`loop()`は使わない） | シンプルな`setup()`/`loop()` |
| 主なlib | Sensor, GPS, PID, SpiLinkMaster | Radio, Actuator, SpiLinkSlave |
| PlatformIO env | `xiao1` | `xiao2` |

XIAO1が姿勢・GPS・誘導PID出力までSPIで計算しきってXIAO2へ送り、XIAO2はそれをそのままモータへ反映しつつ、同じフレームをWiFiで地上局へ中継する（地上局からの手動操作コマンドもXIAO2が直接WiFiで受信し、SPI経由の中継はしない）。

SPI通信の詳細（フレームフォーマット・ピン）は末尾の「[XIAO1 ⇔ XIAO2 通信（SPI）](#xiao1--xiao2-通信spi)」を参照。

---

## XIAO1 タスク構成

```
Core 0                            Core 1（リアルタイム系）
────────                          ──────────────────────────────────
taskGPS  priority 3               taskSensor      priority 5  ← 100Hz 最優先
                                   taskNavigation  priority 4  ← 誘導PID計算
                                   taskSpiLink     priority 3  ← XIAO2への送信
```

Core 1の3タスクはセンサー→誘導PID→SPI送信の依存順に優先度を割り当てている。

| タスク | 周期 | 役割 |
|---|---|---|
| taskSensor | 100Hz | IMU・気圧・地磁気を読み、姿勢フィルタ（重力ベクトルの相補フィルタ＋地磁気チルト補正）を回す。BMM350のODRも100Hzに設定（`lib/Sensor`） |
| taskNavigation | 100Hz | GPS方位とyawの誤差（磁気偏角補正込み）をPIDで補正し、旋回量(`pidOutput`)と目的地方位(`destinationYaw`)をSharedへ書き込む。GPS fix未取得中は出力0固定 |
| taskSpiLink | 100Hz | センサー値・誘導PID出力をXIAO2へ送信する（応答フレームは未使用） |
| taskGPS | イベント駆動 | NMEAをバイト単位でパース。fix有無を`gpsValid`としてSharedへ書き込む |

TODO: `taskStateMachine`（ミッションステート遷移）は未実装。現状`Shared.state`は`STANDBY`のまま固定される。

---

## タスク間データ共有（XIAO1内）

XIAO1内の全タスクが `Shared` 構造体 1つを参照する。mutex で排他制御。XIAO2とはこの構造体を共有しておらず、taskSpiLinkがSPI経由で値をやり取りする。

```
include/shared.h
┌────────────────────────────────────────────────────┐
│ struct SensorData {                                │
│   float alt, roll, pitch, yaw;                     │
│   double lat, lon;                                 │
│   uint32_t timestamp_ms;                           │
│   bool gpsValid;                                   │
│   float pidOutput;       ← taskNavigationが計算    │
│   float destinationYaw;  ← taskNavigationが計算    │
│ };                                                  │
│ struct Shared {                                    │
│   SemaphoreHandle_t mutex;                         │
│   SensorData latest;                               │
│   MissionState state;                              │
│ };                                                  │
└────────────────────────────────────────────────────┘
```

- **書き込み**: taskSensor（alt/roll/pitch/yaw/timestamp_ms）、taskGPS（lat/lon/gpsValid）、taskNavigation（pidOutput/destinationYaw）
- **読み取り**: taskNavigation（yaw/lat/lon/gpsValid）、taskSpiLink（XIAO2への送信用）

---

## ライブラリ構成（lib/）

タスク／`loop()`はlibの薄いラッパーとして機能し、ロジックはすべてlib側に閉じ込める。

```
src/xiao1/tasks/task_xxx.h                    src/xiao2/main.cpp
    │  lib を呼ぶだけ                              │  lib を呼ぶだけ
    ▼                                              ▼
lib/Sensor/          BMP280（気圧）・MPU6050（6軸）・BMM350（地磁気）ドライバ＋姿勢フィルタ   ─ XIAO1
lib/GPS/             TinyGPSPlus をラップしたGPS読み取りクラス（GY-GPSV2-NEO6M）＋方位・距離計算 ─ XIAO1
lib/PID/             汎用PIDコントローラ                                                  ─ XIAO1
lib/SpiLinkMaster/   XIAO2への送信＋応答受信（全二重SPI、標準SPIライブラリを使用）           ─ XIAO1
lib/SpiLinkSlave/    XIAO1からのトランザクション受信（hideakitai/ESP32SPISlaveを使用）      ─ XIAO2
lib/Radio/           WiFi SoftAP 立ち上げ・HTTP GET でバイナリフレーム配信（PULL方式）        ─ XIAO2
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
ESP32-S3（SoftAP: CanSat-AP、XIAO2が担当）
    │  GET http://192.168.4.1/data
    │  37バイト バイナリフレーム（ポーリング間隔 100〜150ms 目安）
    ▼
地上PC（ブラウザ or Pythonスクリプト）
    └─ データ記録・リアルタイム表示
```

WiFi/HTTPは**XIAO2**が担当する（XIAO1はSPI送信のみでWiFiを持たない）。XIAO1がSPIで送った`SpiFrameToXiao2`を、XIAO2がXIAO1のloop()内で`Radio::setData()`にそのまま渡し、リクエストごとに読み直して返す（`Radio::poll()` を呼ぶたびに保留中のHTTPリクエストを処理する）。ブラウザ用の簡易ダッシュボード（`GET /`）も同じ `/data` を`fetch()`でポーリングする。

地上局からのモーター手動制御は `GET /motor?left=-255〜255&right=-255〜255` で受け付ける（PCから機体へのアウトバウンド方向なので、`/data`のPULLと同じく信頼できる方向）。`Radio::getMotorCommandLeft()`/`getMotorCommandRight()` は、`MOTOR_COMMAND_TIMEOUT_MS`（1000ms）以上新しいコマンドが来ていなければ自動的に0を返すフェイルセイフを持つ。地上局側（`ground/receiver.py`）は手動制御が有効な間、250ms間隔でコマンドを送り続けることでこのタイムアウトより十分短い周期を維持する。

このコマンドはXIAO2が直接WiFiで受信し、SPI経由の中継はしない（XIAO1はWiFiを持たないため関与しない）。手動操作と自律PID（XIAO1側で計算した`pid_output`）のどちらを優先するかは、`Radio::hasRecentMotorCommand()`（タイムアウト以内に手動コマンドを受信しているか）で`src/xiao2/main.cpp`の`loop()`が判定する。フラグではなく「直近に手動コマンドが来ているか」で判定するため、`SpiFrameToXiao2`側に調停フラグは持たない。

なお `tests/test_navigation` はGPS方位とBMM350ヘディングの誤差をPIDで補正し左右モーターへ反映する自律航行の統合確認だが、これはSPIを経由しない（Actuatorを直接駆動するスタンドアロンテスト）。`src/xiao1/tasks/task_navigation.h` の誘導PIDロジックと同じ考え方の参考実装として位置づける。

受信バイナリフレームフォーマット（リトルエンディアン、37バイト。`include/spi_protocol.h` の`SpiFrameToXiao2`をそのまま中継しているので詳細はそちら参照）：

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
| 29 | 4 | float32 | pid_output | 誘導PIDの旋回量（-255〜255） |
| 33 | 4 | float32 | destination_yaw | 目的地への方位角 [deg]（磁北基準） |

---

## XIAO1 ⇔ XIAO2 通信（SPI）

XIAO1がマスター、XIAO2がスレーブ。全二重トランザクション1回で双方向のデータを同時にやり取りする。

```
XIAO1（マスター）                              XIAO2（スレーブ）
  taskSpiLink（100Hz）                           loop()
      │  SpiLinkMaster::transfer(out) ──────────►│ SpiLinkSlave::poll(in)
      │  ＝ SPI.transferBytes()（全二重）          │ ＝ ESP32SPISlaveのqueue/trigger方式
      │◄────────────────────────────────────────│ SpiLinkSlave::setResponse(out)
      ▼                                              │
  （応答は未使用）                                     ├─ モータへ反映（pid_output or 手動コマンド）
                                                       └─ Radio::setData()でWiFiテレメトリに反映
```

契約は `include/spi_protocol.h` にある。変更する場合は `lib/SpiLinkMaster` と `lib/SpiLinkSlave` 両方への影響を確認すること。

**XIAO1 → XIAO2（`SpiFrameToXiao2`）**

| フィールド | 型 | 備考 |
|---|---|---|
| timestamp_ms | uint32 | 起動からのms |
| alt / roll / pitch / yaw | float32 | XIAO1のSensorでフュージョン済みの姿勢・高度 |
| lat / lon | float32 | GPS座標（doubleから縮小） |
| mission_state | uint8 | `MissionState`のenum値 |
| pid_output | float32 | `task_navigation.h`が計算した誘導PIDの旋回量（-255〜255） |
| destination_yaw | float32 | `task_navigation.h`が計算した目的地への方位角 [deg]（磁北基準） |

XIAO2はこのフレームをそのままWiFiテレメトリとしても中継する（`lib/Radio`参照）。

**XIAO2 → XIAO1（`SpiFrameFromXiao2`）**

現状XIAO1側で消費するデータが無い（テレメトリはXIAO2が直接WiFiで配信するため）ため、ダミーの1バイトのみ。SPIが全二重方式のため転送自体は必要。

### XIAO2側の処理（モータ駆動・WiFi中継）

`src/xiao2/main.cpp` の `loop()` が、受信した`pid_output`を`BASE_SPEED ± pid_output`の左右差動出力に変換する（誘導・PID自体はXIAO1側の`task_navigation.h`で計算済み）。`Radio::hasRecentMotorCommand()`が真なら地上局からの手動コマンドを優先する。現状のTODO：

- `BASE_SPEED`のミッションステート依存化（現状は固定値150）
- XIAO1からの通信が一定時間途絶えた場合のフェイルセイフ（モータ停止）

### SPIピン・ビルド上の注意

- ピン番号は `include/spi_protocol.h` の `SpiPins` 名前空間で定義（現状は仮値、回路図の実ピンに合わせて要修正）。
- XIAO2は `hideakitai/ESP32SPISlave` ライブラリに依存する（`platformio.ini` の `env:xiao2` にのみ追加）。マスター用（`lib/SpiLinkMaster`）とスレーブ用（`lib/SpiLinkSlave`）を別々のlibフォルダに分けているのは、PlatformIOのライブラリ依存解決がフォルダ単位でソースをコンパイルするため、同じフォルダに同居させるとXIAO1のビルドにもXIAO2専用ライブラリへの依存が混入してしまうことを避けるため。
