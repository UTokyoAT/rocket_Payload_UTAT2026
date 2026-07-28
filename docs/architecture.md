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
| 役割 | センサー読み取り・GPS・誘導フュージョン・PID計算・ミッションステート管理・ニクロム線分離機構 | モーター駆動・WiFi通信（テレメトリ配信・手動操作受信） |
| 実装方式 | デュアルコア＋FreeRTOSタスク（`loop()`は使わない） | シンプルな`setup()`/`loop()` |
| 主なlib | Sensor, GPS, PID, Deployer, SpiLinkMaster | Radio, Actuator, SpiLinkSlave |
| PlatformIO env | `xiao1` | `xiao2` |

XIAO1が姿勢・GPS・誘導PID出力まで計算しきってSPIでXIAO2へ送り、XIAO2はそれをそのままモータへ反映しつつ、同じフレームをWiFiで地上局へ中継する（地上局からの手動操作コマンドもXIAO2が直接WiFiで受信し、SPI経由の中継はしない）。

SPI通信の詳細（フレームフォーマット・ピン）は末尾の「[XIAO1 ⇔ XIAO2 通信（SPI）](#xiao1--xiao2-通信spi)」を参照。

---

## XIAO1 タスク構成

```
Core 0                            Core 1（リアルタイム系）
────────                          ──────────────────────────────────
taskGPS  priority 3               taskSensor      priority 5  ← 100Hz 最優先
                                   taskNavigation  priority 4  ← 誘導PID計算
                                   taskMission     priority 4  ← ミッションステート遷移
                                   taskSpiLink     priority 3  ← XIAO2への送信
```

Core 1の4タスクはセンサー→誘導PID・ミッション判定→SPI送信の依存順に優先度を割り当てている。

| タスク | 周期 | 役割 |
|---|---|---|
| taskSensor | 100Hz | IMU・気圧・地磁気を読み、姿勢フィルタ（重力ベクトルの相補フィルタ＋地磁気チルト補正）を回す。BMM350のODRも100Hzに設定（`lib/Sensor`） |
| taskNavigation | 100Hz | GPS方位とyawの誤差（磁気偏角補正込み）をPIDで補正し、旋回量(`pidOutput`)と目的地方位(`destinationYaw`)をSharedへ書き込む。GPS fix未取得中は出力0固定 |
| taskMission | 100Hz（10ms） | SETTING〜GOAL/ABORTEDのシーケンスを進行させ`Shared.state`を更新し、`lib/Deployer`（ニクロム線・LED）を制御する。詳細は「[ミッションステート遷移](#ミッションステート遷移)」参照 |
| taskSpiLink | 100Hz | センサー値・誘導PID出力・ミッションステートをXIAO2へ送信し、応答フレーム（地上局が設定した目的地）を受け取る |
| taskGPS | イベント駆動 | NMEAをバイト単位でパース。fix有無・衛星数・新規fix到着を`gpsValid`/`gpsSatellites`/`gpsFixSeq`としてSharedへ書き込む |

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
│   int gpsSatellites;      ← taskGPSが計算          │
│   uint32_t gpsFixSeq;     ← taskGPSが計算          │
│   float pidOutput;        ← taskNavigationが計算   │
│   float destinationYaw;   ← taskNavigationが計算   │
│ };                                                  │
│ struct Shared {                                    │
│   SemaphoreHandle_t mutex;                         │
│   SensorData latest;                               │
│   MissionState state;    ← taskMissionが更新       │
│   double goalLat, goalLon; ← taskMission/taskSpiLinkが更新 │
│ };                                                  │
└────────────────────────────────────────────────────┘
```

- **書き込み**: taskSensor（alt/roll/pitch/yaw/timestamp_ms）、taskGPS（lat/lon/gpsValid/gpsSatellites/gpsFixSeq）、taskNavigation（pidOutput/destinationYaw）、taskMission（state/goalLat/goalLon）、taskSpiLink（goalLat/goalLon、地上局が`GET /goal`を送った場合のみ）
- **読み取り**: taskNavigation（yaw/lat/lon/gpsValid/goalLat/goalLon）、taskMission（alt/roll/pitch/gpsValid/gpsSatellites/lat/lon/gpsFixSeq）、taskSpiLink（XIAO2への送信用）

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
lib/Actuator/        モーター（TB6612FNG、3ピン/モーター＋共有STBY方式で左右独立駆動）         ─ XIAO2
lib/Deployer/        ロケット分離・パラシュート分離（いずれもニクロム線でテグス溶断）・LED     ─ XIAO1
lib/DebugLog/        WiFi SoftAP + HTTPでテキストログ配信（tests/配下の単体動作確認専用。本番では未使用）
```

ミッションステート遷移ロジック自体は`lib/`ではなく`src/xiao1/tasks/task_mission.h`に置いている（Sensor/GPS/Deployerなど複数libを横断的に読み書きする調停役のため、他タスクと同じく「libを呼ぶだけの薄いラッパー」ではなく、libをまたぐ状態管理そのものがこのタスクの本体になる）。

---

## ミッションステート遷移

`src/xiao1/tasks/task_mission.h`（`taskMission`、10ms周期）が実装する。実際の運用フローチャートに基づく7状態。「tick」はtaskMission自身の10ms周期を指す（NAVIGATEの停止判定のみ、GPSの実際の更新間隔に合わせて「新規fixの到着回数」で数える。理由は後述）。tick数（LAUNCH_CONFIRM_TICKS等）は50ms周期だった頃の値のまま据え置いているため、各確認時間は5分の1に短縮されている点に注意。

```
SETTING ──衛星10個以上を捕捉──► LAUNCH ──高度8m超を5tick連続検知──► DETACH
   │ 10分タイムアウト                                                  │ deployRocket()
   ▼                                                                    │ 高度変化1m未満を100tickごとに5回連続検知
ABORTED ◄───────────────────────────────────────────────────────────────┤
   ▲                                                                    ▼
   │ 5分タイムアウト                                                 UNFOLD
   │（安定したが20度以内に収まらない）                                  │ deployParachute()
   └──────────────────────────────────────────────────────────────────┤ 変化幅10度以下(直近10tick)
                                                                        │ かつ絶対値20度以下を5tick連続検知
                                                                        ▼
   ABORTED ◄── 10分タイムアウト（停止を検知できず） ──────────── NAVIGATE
      │                                                                 │ 緯度・経度どちらも変化が0.00001度以下を
      │                                                                 │ 新規fixで1回検知→以後5回連続で再確認
      ▼                                                                 ▼
  LED点滅で異常を通知し続ける                                          GOAL ── LED点滅（回収支援）
  （モータはNAVIGATE以外では動かないため自然に停止する）
```

各遷移のトリガーとしきい値（すべて`task_mission.h`冒頭の定数）：

| 遷移 | トリガー |
|---|---|
| SETTING → LAUNCH | GPS衛星捕捉数が`SETTING_MIN_SATELLITES`（10個）以上、かつfix取得済み。このとき現在のGPS座標を`Shared::goalLat/goalLon`（＝打ち上げ地点）として記録する |
| SETTING → ABORTED | `SETTING_TIMEOUT_MS`（10分）経過しても衛星10個に届かない |
| LAUNCH → DETACH | 高度が`LAUNCH_ALT_THRESHOLD_M`（8m）を`LAUNCH_CONFIRM_TICKS`（5tick=0.05秒）連続で超える。タイムアウトなし（発射操作を待ち続ける） |
| DETACH → UNFOLD | 遷移時に`Deployer::deployRocket()`（ロケットから分離）。以後、`DETACH_ALT_SAMPLE_TICKS`（100tick=1秒）ごとにサンプリングした高度変化が`DETACH_ALT_DELTA_THRESHOLD_M`（1m）未満を`DETACH_CONFIRM_TICKS`（5回連続＝計5秒相当）検知＝着地。タイムアウトなし |
| UNFOLD → NAVIGATE | 遷移時に`Deployer::deployParachute()`（パラシュート分離）。直近`UNFOLD_STABLE_WINDOW_TICKS`（10tick=0.1秒）のroll/pitch変化幅がともに`UNFOLD_STABLE_RANGE_DEG`（10度）以下、かつ`abs(roll)`/`abs(pitch)`がともに`UNFOLD_UPRIGHT_ABS_DEG`（20度）以下を`UNFOLD_UPRIGHT_CONFIRM_TICKS`（5tick=0.05秒）連続で検知 |
| UNFOLD → ABORTED | `UNFOLD_TIMEOUT_MS`（5分）経過しても上記条件を満たさない（変化は収まった＝安定したが20度以内に収まらない＝回収不能と判断） |
| NAVIGATE → GOAL | `Shared::goalLat/goalLon`へ向けた自律走行中、緯度・経度**どちらも**変化が新規GPS fixで`NAVIGATE_STOP_DELTA_DEG`（0.00001度）以下であることを1回でも検知したら「停止候補」とし、そこから改めて新規fix`NAVIGATE_STOP_CONFIRM_FIXES`（5回）分すべてが同条件を満たすことを再確認できたら＝停止（到達）。再確認中に1回でも超えたら候補を取り消し、次に条件を満たす新規fixが来るところから数え直す。片方だけで判定すると、進行方向が南北/東西に近いときもう片方の軸がほぼ動かず誤検知するためAND条件にしている |
| NAVIGATE → ABORTED | `NAVIGATE_TIMEOUT_MS`（10分）経過しても停止を検知できない |

**DETACHの着地判定を1tickではなく100tickごとに間引く理由**：1tick（10ms）間隔の生の高度差分は気圧センサーのノイズで振動しやすく、着地していなくても閾値未満に見えて誤検知しかねない。そのため`DETACH_ALT_SAMPLE_TICKS`（100tick=1秒）間隔で高度をサンプリングして差分を取り、そのサンプル単位で`DETACH_CONFIRM_TICKS`（5回）連続して閾値未満なら着地と判定する（着地判定にかかる時間は合計で約5秒）。

**NAVIGATEの停止判定だけ「tick」ではなく「GPS fix到着回数」で数える理由**：GPSは実際には1Hz程度でしか更新されないため、taskMissionの10ms周期でそのまま緯度経度をサンプリングすると、同じ古い値を何度も連続で読んでしまい「移動していない」と誤判定しかねない。そのため`taskGPS`が新規fix受信のたびインクリメントする`SensorData::gpsFixSeq`の変化を検知したときだけ判定する。

**NAVIGATEの停止判定が「1回検知→改めて5回再確認」の2段構えになっている理由**：最初から5回連続一致を要求すると、たまたま最初の数回だけ条件を満たさなかった場合に0からやり直しになり判定が遅れる。そこで1回でも条件を満たした時点をいったん「停止候補」として記録し、そこから独立に5回分の新規fixで再確認する。再確認の途中で1回でも条件を外れたら停止候補を取り消し、次に条件を満たす新規fixが来たところから改めて5回のカウントを始める。

**ABORTED/GOALの共通挙動**：`Deployer::beepPattern()`を呼び続けてLEDを点滅させ、回収支援の位置知らせを継続する。区別はテレメトリの`mission_state`の値で行う。モータはXIAO2側で`MissionState::NAVIGATE`のときしか自律走行しないため（後述）、ABORTED/GOALでは特別な停止処理をせずとも自然にモータが止まる。

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

このコマンドはXIAO2が直接WiFiで受信し、SPI経由の中継はしない（XIAO1はWiFiを持たないため関与しない）。手動操作と自律PID（XIAO1側で計算した`pid_output`）のどちらを優先するかは、`Radio::hasRecentMotorCommand()`（タイムアウト以内に手動コマンドを受信しているか）で`src/xiao2/main.cpp`の`loop()`が判定する。フラグではなく「直近に手動コマンドが来ているか」で判定するため、`SpiFrameToXiao2`側に調停フラグは持たない。ただし自律PID出力（`else if`側）は`mission_state`が`NAVIGATE`のときしか反映されない（手動操作はステートに関わらず常に優先）。詳細は「[ミッションステート遷移](#ミッションステート遷移)」参照。

誘導PIDの目的地座標の既定値は、SETTINGシーケンス完了時に`taskMission`が取得したGPS座標（＝打ち上げ地点）で、以後NAVIGATEはこの地点へ戻る形（return-to-launch）で走行する。地上局はこれを`GET /goal?lat=..&lon=..`でいつでも上書きできる。モーターコマンドと異なりタイムアウトで無効化されない（通信が途切れたからといって目的地を失わせるのは危険なため、明示的に上書きされるまで保持する）。この座標はXIAO1側の誘導計算（`task_navigation.h`）で使うため、XIAO2が受信した値をSPIの応答フレーム（`SpiFrameFromXiao2`、XIAO2→XIAO1方向）に載せてXIAO1へ送り返す。`ground/receiver.py`は`--dest-lat`/`--dest-lon`起動引数を指定すると、地図表示用の目的地としてだけでなくこの`/goal`エンドポイントにも同じ座標を送るため、地上局の表示上の目的地と機体が実際に向かう先が食い違わない。

誘導PIDの統合動作確認は本番のXIAO1+XIAO2ペア（`env:xiao1`/`env:xiao2`をSPI接続）で行うのが基本だが、`tests/test_navigation`はXIAO1単体で誘導ロジックだけを素早く確認するための補助テストとして別途用意している。Sensor・GPSに加え、本来XIAO2側にしかないモーターをベンチ確認用に直結する（`lib/Actuator`はXIAO2の実配線＝I2C/GPSピンと衝突するため使えず、空いているD0-D3で簡易2ピン方式の仮配線にする）。`lib/Radio`を流用し、`GET /goal?lat=..&lon=..`で受け取った目的地へ、GPS fix取得済みの間だけPID走行する（fix未取得・目的地未受信の間はモーターを動かさない）。テレメトリは本番と同じ`SpiFrameToXiao2`レイアウトで`/data`から見られる。

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
  Shared::goalLat/goalLonを更新                        ├─ モータへ反映（pid_output or 手動コマンド）
  （goal_valid=1のときのみ）                            └─ Radio::setData()でWiFiテレメトリに反映
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

| フィールド | 型 | 備考 |
|---|---|---|
| goal_valid | uint8 | 地上局が`GET /goal`を一度でも送っていれば1。0の間は`goal_lat/goal_lon`は不定値でXIAO1側は無視する |
| goal_lat / goal_lon | float32 | 地上局が`GET /goal?lat=..&lon=..`で設定した目的地座標 |

`taskSpiLink`が`goal_valid`を見て、真の場合のみ`Shared::goalLat/goalLon`を上書きする（未設定の間は`shared.h`の既定値のまま）。

### XIAO2側の処理（モータ駆動・WiFi中継）

`src/xiao2/main.cpp` の `loop()` が、受信した`pid_output`を`BASE_SPEED ± pid_output`の左右差動出力に変換する（誘導・PID自体はXIAO1側の`task_navigation.h`で計算済み）。ただしこの自律PID出力は`mission_state`が`MissionState::NAVIGATE`のときしか反映しない（発射・分離・パラシュート展開の最中にモータが勝手に動き出さないための安全ゲート）。`Radio::hasRecentMotorCommand()`が真なら、ミッションステートに関わらず地上局からの手動コマンドを優先する。さらにXIAO1からのSPIフレームが`SPI_LINK_TIMEOUT_MS`（5秒）以上途絶えた場合は、XIAO1側の異常とみなし手動コマンドより優先してモータを強制停止する（`lastSpiFrameMs`で最終受信時刻を追跡）。優先順位は「SPI断 > 手動コマンド > NAVIGATE時の自律PID > それ以外は停止」。現状のTODO：

- `BASE_SPEED`のミッションステート依存化（現状は固定値150）

### SPIピン・ビルド上の注意

- ピン番号は `include/spi_protocol.h` の `SpiPins` 名前空間で定義。SCK/MISO/MOSIはXIAO1・XIAO2共通（D8/D9/D10）だが、CSはXIAO1側`CS_MASTER`（D0）とXIAO2側`CS_SLAVE`（D7）で異なるGPIOに配線されているため分けて定義している。
- XIAO2は `hideakitai/ESP32SPISlave` ライブラリに依存する（`platformio.ini` の `env:xiao2` にのみ追加）。マスター用（`lib/SpiLinkMaster`）とスレーブ用（`lib/SpiLinkSlave`）を別々のlibフォルダに分けているのは、PlatformIOのライブラリ依存解決がフォルダ単位でソースをコンパイルするため、同じフォルダに同居させるとXIAO1のビルドにもXIAO2専用ライブラリへの依存が混入してしまうことを避けるため。
