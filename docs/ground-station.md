# 地上局セットアップ

## 構成

テレメトリはPULL方式: 地上PC側（ブラウザ／Pythonスクリプト）が機体の `GET /data` を定期的にポーリングしにいく。機体からPCへ能動的にデータを送りつけることはしない（PC側ファイアウォールに阻まれ信頼できないため）。

モーター手動制御はその逆方向（PC→機体のアウトバウンド `GET /motor?value=N`）で、こちらは`/data`のPULLと同じくPC側から能動的に接続するので信頼できる。機体側は1秒間コマンドを受信しないと自動的に出力を0にするフェイルセイフを持つ。

```
ESP32-S3（CanSat）                  地上PC
──────────────────                 ─────────────────────────────────────
WiFi SoftAP                        1. CanSat-AP に接続
  SSID: CanSat-AP   ←── 接続 ──
  IP:   192.168.4.1

GET /       ──── HTML ──────────► 2. ブラウザで 192.168.4.1 を開く
GET /data   ──── 31byte frame ──► 3. ダッシュボードが150ms間隔でポーリング表示
GET /data   ──── 31byte frame ──► 4. receiver.py が100〜150ms間隔でポーリングし CSV に保存
GET /motor?value=N ◄── ok ──────  5. receiver.py の手動制御スライダーが250ms間隔で送信
```

---

## ブラウザダッシュボード

追加ソフト不要。ESP32-S3 から HTML を配信するため、接続後にブラウザで開くだけで使える。

1. PCの WiFi を **CanSat-AP** に接続（パスワード: `cansat2026`）
2. ブラウザで `http://192.168.4.1` を開く

表示内容：
- ミッションステート
- 高度・姿勢（Roll / Pitch / Yaw）
- GPS座標
- モーター出力
- 高度の時系列グラフ（直近300点）

ページは`/data`を150ms間隔でGETポーリングしており、取得に失敗すると状態表示が「● unreachable」になる（自動的にポーリングを継続し、次に成功すれば「● reachable」に戻る）。目的地までの距離・方位・地図表示・モーター手動制御はブラウザ版では提供していない（Pythonレシーバー側のみ）。

---

## Python レシーバー（CSV ロギング＋GUI）

### セットアップ

```bash
cd ground
pip install -r requirements.txt   # GUIの地図描画に matplotlib を使用
```

GUI表示には Tkinter（python.org配布のWindows版インストーラに標準同梱）と matplotlib を使用する。

### 実行

```bash
python receiver.py
```

- Tkinterウィンドウが開き、以下をリアルタイム表示する：
  - ミッションステート・高度・姿勢（Roll/Pitch/Yaw）・GPS座標
  - モーター出力（数値＋バーゲージ、-255〜255）
  - 高度の時系列チャート
  - 現在地・機体の向き（矢印）・移動軌跡・東西南北を表示するマップ（matplotlib）
  - 目的地までの距離・方位（`--dest-lat`/`--dest-lon` 指定時のみ）
  - 手動モーター制御パネル（下記参照）
- `ground/logs/log_YYYYMMDD_HHMMSS.csv` に自動保存
- ターミナルにも1行readoutを表示する
- GET取得に失敗しても落ちずに警告を出しながらポーリングを継続する
- ウィンドウを閉じる、または `Ctrl+C` で停止

#### 手動モーター制御

「有効にする」チェックボックスをオンにすると、スライダー（-255〜255）の現在値を `GET /motor?value=N` として250ms間隔で機体に送り続ける。チェックを外す・STOPボタンを押す・ウィンドウを閉じる、のいずれでも即座に0を送信する。

安全機構（多重化）：
1. チェックを外す/STOP/終了時に明示的に0を送信
2. チェックを外すとスライダーは無効化され、それ以上コマンドを送らなくなる
3. 機体側（`Radio::getMotorCommand()`）は1秒間新しいコマンドを受信しないと自動的に出力を0にする（WiFi切断や地上局クラッシュ時のフェイルセイフ）

チェックを入れた直後は実際にモーターが動くので、卓上テスト時はモーターへの配線状態に注意すること。

目的地を指定すると、マップ上に目的地（★マーク）が表示され、現在地からの距離・方位も表示される：

```bash
python receiver.py --dest-lat 35.6820 --dest-lon 139.7670
```

目的地を指定しない場合、マップは最初に受信した位置を原点として機体の軌跡・向きのみを表示する（東西南北の向きは変わらない）。

Tkinter/matplotlibが使えない環境向けに、GUIなしのターミナル＋CSVのみのモードもある：

```bash
python receiver.py --headless
```

接続先IPやポーリング間隔を変えたい場合：

```bash
python receiver.py --host 192.168.4.1 --interval-ms 150
```

### CSV フォーマット

| 列 | 内容 | 単位 |
|---|---|---|
| timestamp_ms | ESP32-S3 起動からの経過時間 | ms |
| alt_m | 高度 | m |
| roll_deg | ロール角 | deg |
| pitch_deg | ピッチ角 | deg |
| yaw_deg | ヨー角 | deg |
| lat | 緯度 | deg |
| lon | 経度 | deg |
| state | ミッションステート名 | - |
| motor_output | モーター出力 | -255〜255 |
| dist_to_dest_m | 目的地までの距離（`--dest-lat`/`--dest-lon`未指定時は空欄） | m |
| bearing_to_dest_deg | 目的地への方位（同上、未指定時は空欄） | deg |

### ターミナル表示例

```
Polling http://192.168.4.1:80/data every 120ms ...
Destination: 35.682000, 139.767000
Logging to logs/log_20260101_120000.csv

[ASCENDING   ] alt= 120.30m  R=  -3.1°  P=   1.8°  Y= 275.0°  GPS=35.68124,139.76713  M=   0  DIST=  102.4m BRG= 34.2°
```

### 実機なしでの動作確認（モックデバイス）

実機が手元になくても、`mock_device.py` を使って `receiver.py` の動作を確認できる。

```bash
# ターミナル1
python mock_device.py

# ターミナル2
python receiver.py --host 127.0.0.1 --port 8000 --dest-lat 35.6820 --dest-lon 139.7670
```

`mock_device.py` は正弦波で変化するダミーの31バイトフレームを `http://127.0.0.1:8000/data` に配信し続ける。`GET /motor?value=N` も受け付け、機体と同じ1秒フェイルセイフ付きで`motor_output`に折り返すため、GUIのスライダーを動かして手動制御パネルの動作を実機なしで確認できる。

---

## バイナリフレームフォーマット（参考）

ESP32-S3 が `/data` で配信する31バイトのバイナリフレーム（リトルエンディアン）。`lib/Radio/Radio.h`・`ground/receiver.py`・`lib/Radio/dashboard.h` の3箇所で共有する契約。Python側フォーマット文字列は `"<IffffffBh"`。

| Offset | Size | 型 | フィールド | 備考 |
|---|---|---|---|---|
| 0 | 4 | uint32 | timestamp_ms | 起動からのms |
| 4 | 4 | float32 | alt | 高度 [m] |
| 8 | 4 | float32 | roll | ロール角 [deg] |
| 12 | 4 | float32 | pitch | ピッチ角 [deg] |
| 16 | 4 | float32 | yaw | ヨー角 [deg] |
| 20 | 4 | float32 | lat | 緯度（機体側でdoubleから縮小） |
| 24 | 4 | float32 | lon | 経度（機体側でdoubleから縮小） |
| 28 | 1 | uint8 | mission_state | 下表参照 |
| 29 | 2 | int16 | motor_output | `Actuator::setMotor()`相当値（-255〜255）。地上局の手動制御（`GET /motor`）で受信した値をそのまま反映。自律ナビゲーションロジックは未実装 |

`mission_state` の値と対応するステート：

| 値 | ステート |
|---|---|
| 0 | STANDBY |
| 1 | ASCENDING |
| 2 | DESCENDING |
| 3 | SEPARATING |
| 4 | RUNNING |
| 5 | GOAL |
