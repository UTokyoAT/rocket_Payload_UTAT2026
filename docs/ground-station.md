# 地上局セットアップ

## 構成

テレメトリはPULL方式: 地上PC側（ブラウザ／Pythonスクリプト）が機体の `GET /data` を定期的にポーリングしにいく。機体からPCへ能動的にデータを送りつけることはしない（PC側ファイアウォールに阻まれ信頼できないため）。

モーター手動制御はその逆方向（PC→機体のアウトバウンド `GET /motor?left=N&right=M`）で、こちらは`/data`のPULLと同じくPC側から能動的に接続するので信頼できる。左右独立に指定し、機体側は1秒間コマンドを受信しないと自動的に両方の出力を0にするフェイルセイフを持つ。ただし自律PID走行はミッションステートが`NAVIGATE`のときしか反映されない（`src/xiao2/main.cpp`参照）ため、手動操作もその他の状態では実質的にモータを直接動かすだけになる。

誘導PIDの目的地座標も同じ方向（`GET /goal?lat=..&lon=..`）で設定できる。モーターコマンドと異なりタイムアウトでは無効化されず、明示的に上書きされるまで保持される。既定ではSETTINGシーケンス完了時に機体が取得したGPS座標（打ち上げ地点）が目的地になるが、`/goal`を送ればいつでも優先的に上書きできる。

```
ESP32-S3（CanSat）                  地上PC
──────────────────                 ─────────────────────────────────────
WiFi SoftAP                        1. CanSat-AP に接続
  SSID: CanSat-AP   ←── 接続 ──
  IP:   192.168.4.1

GET /       ──── HTML ──────────► 2. ブラウザで 192.168.4.1 を開く
GET /data   ──── 37byte frame ──► 3. ダッシュボードが150ms間隔でポーリング表示
GET /data   ──── 37byte frame ──► 4. receiver.py が100〜150ms間隔でポーリングし CSV に保存
GET /motor?left=N&right=M ◄── ok ─ 5. receiver.py の左右手動制御スライダーが250ms間隔で送信
GET /goal?lat=..&lon=..   ◄── ok ─ 6. receiver.py が --dest-lat/--dest-lon 指定時に起動時1回送信
```

---

## 単体動作確認テスト（tests/）のデバッグ出力

`tests/` 配下の各動作確認コード（`test_bmm350`・`test_bmp280`・`test_gps`・`test_i2c_scan`・`test_motor`・`test_mpu6050`）は、USBシリアルではなく **WiFi経由** でログを出す（`lib/DebugLog`）。卓上でUSBケーブルを挿さなくても、PCやスマホから確認できる。

1. PCのWiFiを **CanSat-AP** に接続（パスワード: `cansat2026`）
2. ブラウザで `http://192.168.4.1` を開く（300ms間隔で自動更新される簡易ログビューア）
   - または `GET http://192.168.4.1/log` をポーリングすればプレーンテキストで直近60行が取れる（`curl`や自作スクリプトから見る場合はこちら）

本番ファームウェア（XIAO2の`lib/Radio`）と同じSSID/パスワードを使っているため、複数の基板・テストを同時にAPとして起動しないこと（同じSSIDが衝突する。1枚ずつ書き込んで確認する運用を前提にしている）。

`Serial.begin(115200)`自体は残しているため、USBシリアルを接続していれば`DebugLog::printf()`の内容はそちらにも同時出力される（WiFiが使えない環境でのフォールバック）。

---

## test_navigation（XIAO1単体の誘導PID確認）

`tests/test_navigation`はXIAO1単体で誘導ロジックだけを素早く確認するためのテスト（`lib/DebugLog`ではなく`lib/Radio`を流用しており、本番の`/data`と同じフレームで見える）。

1. PCのWiFiを **CanSat-AP** に接続（パスワード: `cansat2026`）
2. `GET http://192.168.4.1/goal?lat=..&lon=..` で目的地を送る（ブラウザのアドレスバーでも`curl`でもよい）
3. ブラウザで `http://192.168.4.1` を開くか`receiver.py --host 192.168.4.1`で`pid_output`・`destination_yaw`・`yaw`・GPS座標を確認する

GPS fixを取得できていない、または目的地を一度も送っていない間はモーターが動かない（`pid_output`は0のまま）。動作確認するモーターは本番のXIAO2用配線ではなく、このテスト専用にD0-D3へ仮配線する（`tests/test_navigation/main.cpp`冒頭のコメント参照）。まずは車輪を浮かせた状態で確認すること。

---

## ブラウザダッシュボード

追加ソフト不要。ESP32-S3 から HTML を配信するため、接続後にブラウザで開くだけで使える。

1. PCの WiFi を **CanSat-AP** に接続（パスワード: `cansat2026`）
2. ブラウザで `http://192.168.4.1` を開く

表示内容：
- ミッションステート
- 高度・姿勢（Roll / Pitch / Yaw）
- GPS座標
- 誘導PID出力・目的地方位
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
  - 誘導PID出力（数値＋バーゲージ、-255〜255）・目的地方位（磁北基準）
  - 高度の時系列チャート
  - 現在地・機体の向き（矢印）・移動軌跡・東西南北を表示するマップ（matplotlib）
  - 目的地までの距離・方位（`--dest-lat`/`--dest-lon` 指定時のみ）
  - 左右独立の手動モーター制御パネル（下記参照）
- `ground/logs/log_YYYYMMDD_HHMMSS.csv` に自動保存
- ターミナルにも1行readoutを表示する
- GET取得に失敗しても落ちずに警告を出しながらポーリングを継続する
- ウィンドウを閉じる、または `Ctrl+C` で停止

#### 手動モーター制御（左右独立）

「有効にする」チェックボックスをオンにすると、LEFT/RIGHT各スライダー（-255〜255）の現在値を `GET /motor?left=N&right=M` として250ms間隔で機体に送り続ける。チェックを外す・STOPボタンを押す・ウィンドウを閉じる、のいずれでも即座に両方0を送信する。

安全機構（多重化）：
1. チェックを外す/STOP/終了時に明示的に left=0, right=0 を送信
2. チェックを外すと両スライダーは無効化され、それ以上コマンドを送らなくなる
3. 機体側（`Radio::getMotorCommandLeft()`/`getMotorCommandRight()`）は1秒間新しいコマンドを受信しないと自動的に両方の出力を0にする（WiFi切断や地上局クラッシュ時のフェイルセイフ）

チェックを入れた直後は実際にモーターが動くので、卓上テスト時はモーターへの配線状態に注意すること。

目的地を指定すると、マップ上に目的地（★マーク）が表示され、現在地からの距離・方位も表示される：

```bash
python receiver.py --dest-lat 35.6820 --dest-lon 139.7670
```

目的地を指定しない場合、マップは最初に受信した位置を原点として機体の軌跡・向きのみを表示する（東西南北の向きは変わらない）。

Tkinter/matplotlibが使えない環境向けに、GUIなしのターミナル＋CSVのみのモードもある（手動モーター制御はGUI専用のためheadlessでは使えない）：

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
| pid_output | 誘導PIDの旋回量 | -255〜255 |
| destination_yaw_deg | 目的地への方位角（機体側計算、磁北基準） | deg |
| dist_to_dest_m | 目的地までの距離（`--dest-lat`/`--dest-lon`未指定時は空欄） | m |
| bearing_to_dest_deg | 目的地への方位（同上、未指定時は空欄） | deg |

### ターミナル表示例

```
Polling http://192.168.4.1:80/data every 120ms ...
Destination: 35.682000, 139.767000
Logging to logs/log_20260101_120000.csv

[NAVIGATE    ] alt= 120.30m  R=  -3.1°  P=   1.8°  Y= 275.0°  GPS=35.68124,139.76713  PID=   0.0  DESTYAW=  34.2°  DIST=  102.4m BRG= 34.2°
```

### 実機なしでの動作確認（モックデバイス）

実機が手元になくても、`mock_device.py` を使って `receiver.py` の動作を確認できる。

```bash
# ターミナル1
python mock_device.py

# ターミナル2
python receiver.py --host 127.0.0.1 --port 8000 --dest-lat 35.6820 --dest-lon 139.7670
```

`mock_device.py` は正弦波で変化するダミーの37バイトフレームを `http://127.0.0.1:8000/data` に配信し続ける。`GET /motor?left=N&right=M` も受け付け、機体と同じ1秒フェイルセイフ付きで`pid_output`（left側の値）に折り返すため、GUIの左右スライダーを動かして手動制御パネルの動作を実機なしで確認できる。`GET /goal?lat=..&lon=..` も200 okを返すだけの受け口として用意してあり、`--dest-lat`/`--dest-lon`指定時に`receiver.py`が自動送信する`GoalSender`が無限リトライし続けないようにしている。

---

## バイナリフレームフォーマット（参考）

ESP32-S3（XIAO2）が `/data` で配信する37バイトのバイナリフレーム（リトルエンディアン）。XIAO1がSPIで送った`SpiFrameToXiao2`（`include/spi_protocol.h`）をXIAO2がそのまま中継したもので、`lib/Radio/Radio.h`・`ground/receiver.py`・`lib/Radio/dashboard.h` の3箇所で共有する契約。Python側フォーマット文字列は `"<IffffffBff"`。

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
| 29 | 4 | float32 | pid_output | 誘導PIDの旋回量（-255〜255）。地上局の手動制御（`GET /motor`）が有効な間はXIAO2がそちらを優先してモータへ反映する |
| 33 | 4 | float32 | destination_yaw | 目的地への方位角 [deg]（磁北基準、XIAO1が計算） |

`mission_state` の値と対応するステート（詳細は `docs/architecture.md` の「ミッションステート遷移」参照）：

| 値 | ステート |
|---|---|
| 0 | SETTING |
| 1 | LAUNCH |
| 2 | DETACH |
| 3 | UNFOLD |
| 4 | NAVIGATE |
| 5 | GOAL |
| 6 | ABORTED |
