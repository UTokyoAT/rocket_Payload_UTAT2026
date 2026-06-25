# 制御構造・アーキテクチャ

## 概要

ESP32（WROVER）のデュアルコア＋FreeRTOSを活用し、センサー読み取り・状態遷移・WiFi通信を並列で実行する。`loop()`は使わず、すべてをFreeRTOSタスクに委ねる構成。

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
| taskWifi | 20Hz | 最新センサーデータをWebSocketでブロードキャスト |

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
- **読み取り**: taskWifi（ブロードキャスト用）、taskStateMachine（判定用）

---

## ライブラリ構成（lib/）

タスクはlibの薄いラッパーとして機能し、ロジックはすべてlib側に閉じ込める。

```
src/tasks/task_xxx.h
    │  lib を呼ぶだけ
    ▼
lib/Sensor/          BMP388・ICM-42688・QMC5883 ドライバ＋姿勢フィルタ
lib/GPS/             TinyGPSPlus をラップしたGPS読み取りクラス
lib/Radio/           WiFi SoftAP 立ち上げ・WebSocket ブロードキャスト
lib/Actuator/        モーター（PWM）・パラシュート・分離・ブザー・LED
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
                              FALLING
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
| ASCENDING → FALLING | 高度減少 or IMU衝撃加速度検知 |
| FALLING → SEPARATING | 低高度到達 |
| SEPARATING → RUNNING | 分離処理完了後即時 |
| RUNNING → GOAL | タイムアウト |

---

## 無線通信

地上PCとはWiFi SoftAPで常時接続し、WebSocketで20Hzストリーミング。SDカードは使用しない。

```
ESP32（SoftAP: CanSat-AP）
    │  WebSocket ws://192.168.4.1/ws
    │  20Hz JSON ストリーム
    ▼
地上PC（ブラウザ or Pythonスクリプト）
    └─ データ記録・リアルタイム表示
```

受信JSONフォーマット：

```json
{
  "t":    12345,    // timestamp [ms]
  "alt":  120.5,    // 高度 [m]
  "roll": -3.2,     // ロール角 [deg]
  "pitch": 1.8,     // ピッチ角 [deg]
  "yaw":  275.0,    // ヨー角 [deg]
  "lat":  35.681236,
  "lon":  139.767125,
  "state": 3        // MissionState の enum値
}
```
