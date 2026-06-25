# 地上局セットアップ

## 構成

```
ESP32（CanSat）                    地上PC
──────────────────                 ─────────────────────────────────────
WiFi SoftAP                        1. CanSat-AP に接続
  SSID: CanSat-AP   ←── 接続 ──
  IP:   192.168.4.1

HTTP  GET /          ──── HTML ──► 2. ブラウザで 192.168.4.1 を開く
WebSocket /ws        ──── JSON ──► 3. ダッシュボードがリアルタイム表示
                     ──── JSON ──► 4. receiver.py が CSV に保存
```

---

## ブラウザダッシュボード

追加ソフト不要。ESP32 から HTML を配信するため、接続後にブラウザで開くだけで使える。

1. PCの WiFi を **CanSat-AP** に接続（パスワード: `cansat2026`）
2. ブラウザで `http://192.168.4.1` を開く

表示内容：
- ミッションステート
- 高度・姿勢（Roll / Pitch / Yaw）
- GPS座標
- 高度の時系列グラフ（直近300点）

接続が切れた場合は2秒後に自動再接続する。

---

## Python レシーバー（CSV ロギング）

### セットアップ

```bash
cd ground
pip install -r requirements.txt
```

### 実行

```bash
python receiver.py
```

- `ground/logs/log_YYYYMMDD_HHMMSS.csv` に自動保存
- 接続が切れた場合は3秒後に自動再接続
- `Ctrl+C` で停止

### CSV フォーマット

| 列 | 内容 | 単位 |
|---|---|---|
| timestamp_ms | ESP32 起動からの経過時間 | ms |
| alt_m | 高度 | m |
| roll_deg | ロール角 | deg |
| pitch_deg | ピッチ角 | deg |
| yaw_deg | ヨー角 | deg |
| lat | 緯度 | deg |
| lon | 経度 | deg |
| state | ミッションステート名 | - |

### ターミナル表示例

```
Connected. Logging to logs/log_20260101_120000.csv

[ASCENDING   ] alt= 120.30m  R=  -3.1°  P=   1.8°  Y= 275.0°  GPS=35.68124,139.76713
```

---

## WebSocket JSON フォーマット（参考）

ESP32 が 20Hz で送信するメッセージ：

```json
{
  "t":     12345,
  "alt":   120.5,
  "roll":  -3.2,
  "pitch":  1.8,
  "yaw":  275.0,
  "lat":   35.681236,
  "lon":  139.767125,
  "state": 1
}
```

`state` の値と対応するステート：

| 値 | ステート |
|---|---|
| 0 | STANDBY |
| 1 | ASCENDING |
| 2 | FALLING |
| 3 | SEPARATING |
| 4 | RUNNING |
| 5 | GOAL |
