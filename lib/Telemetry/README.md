# Telemetry ライブラリ

Wi-Fi(STA) + MQTT で地上局(`ground/server` の Mosquitto)へデータを送るライブラリ。
`Telemetry` クラスが接続管理と送信APIを、`TelemetryFormat` がJSONペイロードの生成を担当する。

対応するサーバー側の説明・MQTTトピック一覧・動作確認方法は [`ground/server/README.md`](../../ground/server/README.md) を参照。

## できること(3つの送信API)

| API | QoS | トピック | 用途 |
|---|---|---|---|
| `publishRecord()` | 1 | `<prefix>/telemetry` | 本番用。統合レコード(センサー全部入り)を送る。10Hz想定 |
| `sendTest()` | 0 | `<prefix>/test/<name>` | 切り分けテスト用。好きなフィールドだけ送る |
| `log()` | 0 | `<prefix>/log` | テキスト1行のログ。Serialにも出る |

`begin()` は接続を待たずに戻る。Wi-Fi/MQTTの再接続は内部タスク(Core 0で動作)が自動でやるので、
呼び出し側は上の送信APIを呼ぶだけでよい。送信APIはどのタスク/コアから呼んでも構わない。

## 基本的な使い方

```cpp
#include <Telemetry.h>

static Telemetry telemetry;

void setup() {
    Telemetry::Config cfg;
    cfg.ssid     = "your-ssid";
    cfg.password = "your-password";   // オープンAPなら nullptr のまま
    cfg.mqttHost = "192.168.x.x";     // Mosquittoを動かしているPCのIP
    telemetry.begin(cfg);
}

void loop() {
    TelemetryRecord rec;
    rec.baro.pressure_hpa = 1013.2f;
    rec.baro.alt_m        = 12.1f;
    // 取得できていないフィールドは NAN (int系は負値) のままでよい → JSONから省略される
    telemetry.publishRecord(rec);
}
```

具体例は [`tests/test_telemetry/main.cpp`](../../tests/test_telemetry/main.cpp) も参照。

## `Telemetry::Config`(`begin()` に渡す接続設定)

| フィールド | 意味 |
|---|---|
| `ssid` | 接続先Wi-FiのSSID。**必須**(`nullptr`だと`begin()`が`false`を返す) |
| `password` | Wi-Fiパスワード。オープンAPなら `nullptr` |
| `mqttHost` | Mosquittoを動かしているPCのIPアドレス。**必須** |
| `mqttPort` | MQTTブローカーのポート。デフォルト `1883` |
| `clientId` | MQTTクライアントID。デフォルト `"xiao-esp32s3"` |
| `topicPrefix` | トピックの先頭(`<prefix>/telemetry` など)。デフォルト `"rocket"`(`ground/server/telegraf/telegraf.conf` が `rocket/...` を購読しているので、変える場合はサーバー側も合わせること) |

## `Telemetry` クラスのメソッド

### `bool begin(const Config& cfg)`
Wi-Fi接続とMQTT管理タスクを開始する。渡した文字列(`ssid`など)は内部にコピーされるので、
呼び出し後に元の変数が破棄されても問題ない。
- `ssid` か `mqttHost` が `nullptr` → `false`
- 既に `begin()` 済み(二重呼び出し) → `false`
- 内部タスクの生成に失敗 → `false`
- 接続完了は待たない(呼んだ直後は未接続)。実際に繋がったかは `isConnected()` で確認する。

### `bool isConnected() const`
MQTTブローカーに接続済みかを返す。

### `bool publishRecord(const TelemetryRecord& record)`
本番用の統合レコードをQoS1で送信する。
- `seq`(送信するたびに1つずつ増える連番)と `uptime_ms`(起動からの経過時間, ms)はライブラリが自動で付与する。両方とも起動のたびに0から数え直す。
- 切断中でも呼べる。RAM上のoutboxに積んでおいて再接続後に再送する(QoS1のライブラリ内挙動)。ただし空きヒープが16KBを切ると新規のキューイングを拒否して`false`を返す。電源断・再起動でoutboxの中身は失われる。
- `record` の中で値が `NAN`(GNSSの`fix`/`sats`は負値)のフィールドは「未取得」として送信JSONから省略される。センサーが壊れていても他のフィールドだけ送られる。

### `bool sendTest(const char* name, std::initializer_list<TelemetryField> fields)`
配線確認・センサー単体テストなど、切り分け用に任意のフィールドだけを送る。
- `name`: トピック名の一部になる(`<prefix>/test/<name>`)。英数字・`_`・`-` 以外の文字は `_` に置換される。
- `fields`: `{"キー名", 値}` のペアを好きなだけ並べる。数値型なら何でも渡せる(`int`/`float`/`double`/`bool`など)。
  ```cpp
  telemetry.sendTest("bmp280", {{"pressure_hpa", p}, {"alt_m", a}});
  ```
- `NAN` を渡したフィールドは出力から省略される。
- 未接続の場合は何もせず `false` を返す(`publishRecord`と違いoutboxには積まれない)。

### `void log(const char* fmt, ...)`
`printf`形式で1行のログメッセージを組み立て、`<prefix>/log` に送る(QoS0)。
- 必ずSerialにも出力される(`[経過ms] メッセージ`の形式)。
- 未接続の場合はSerial出力のみでMQTT送信はスキップされる。
- メッセージ本体は160文字、Serial出力込みの行は192文字でそれぞれ切り詰められる。

## `TelemetryRecord`(統合レコードの中身)

`publishRecord()` に渡す構造体。各グループのフィールドはすべて未取得時は `NAN`(`TelemetryGnss`の`fix`/`sats`のみ負値)がデフォルトになっているので、取得できたものだけ埋めればよい。1つもフィールドが有限値でないグループはJSON自体から丸ごと省略される。

| グループ | 型 | フィールド | 意味 |
|---|---|---|---|
| `pos` | `TelemetryPos` | `lat`, `lon`(double), `alt`(float) | 緯度・経度・高度 |
| `imu` | `TelemetryImu` | `ax,ay,az,gx,gy,gz`(float) | 加速度(a)・角速度(g)のXYZ |
| `baro` | `TelemetryBaro` | `pressure_hpa`, `alt_m`(float) | 気圧センサーの気圧・高度 |
| `encoder` | `TelemetryEncoder` | `right_rev`, `left_rev`(double) | 左右ホイールエンコーダーの累積回転数 |
| `gnss` | `TelemetryGnss` | `fix`, `sats`(int, 未取得は負値), `hdop`(float) | GNSSのFix種別・衛星数・HDOP |
| `ekf` | `TelemetryEkf` | `position_N`, `position_E`, `speed`, `azimuth`, `bias_accel`, `bias_gyro`(float) | EKF(拡張カルマンフィルタ)推定値。北方向/東方向位置・速度・方位角・バイアス推定値 |

## `TelemetryField`(`sendTest()` 用の任意フィールド)

```cpp
struct TelemetryField {
    const char* key;   // JSONのキー名
    double      value; // 値(内部でdoubleに変換される)
    bool        isInt;   // 整数型(bool含む)なら小数点なしで出力
    bool        isFloat; // floatなら有効数字7桁、doubleなら15桁で出力
};
```
コンストラクタに `{"キー名", 値}` の形で渡すだけでよく、`isInt`/`isFloat` は渡した値の型から自動判定される。
呼び出し側が明示的に構築する必要はない(`sendTest`の`fields`引数として使う想定)。

## `TelemetryFormat`(JSON生成部、単体テスト可能)

Arduino非依存になっており、PC上でも単体検証できるよう`Telemetry`本体から分離されている。`Telemetry`クラスが内部で呼んでいるだけなので、通常は直接使う必要はない。

### `size_t telemetryFormatRecord(char* buf, size_t cap, const TelemetryRecord& r, uint32_t seq, uint32_t uptime_ms)`
`TelemetryRecord` を `{"seq":N,"uptime_ms":M,"pos":{...},"imu":{...},...}` 形式のJSON文字列にする。
- `buf`/`cap`: 書き込み先バッファとその容量。
- `seq`/`uptime_ms`: 呼び出し側(`Telemetry::publishRecord`)が採番して渡す。
- 戻り値: 書き込んだ長さ(終端の`\0`を除く)。バッファ不足の場合は `0`。

### `size_t telemetryFormatFields(char* buf, size_t cap, const TelemetryField* fields, size_t count, uint32_t uptime_ms)`
`TelemetryField` の配列を `{"uptime_ms":N,"key1":v1,"key2":v2,...}` 形式のJSON文字列にする。
- `NAN` のフィールドは出力から省略される。
- 戻り値の意味は `telemetryFormatRecord` と同じ。

## 送信されるJSONの数値の丸め方(参考)

| 項目 | 出力形式 |
|---|---|
| `pos.lat`/`lon` | 小数点以下7桁 |
| `pos.alt` | 小数点以下2桁 |
| `imu.*` | 小数点以下4桁 |
| `baro.*` | 小数点以下2桁 |
| `encoder.*` | 小数点以下7桁 |
| `gnss.hdop` | 小数点以下2桁(`fix`/`sats`は整数) |
| `ekf.position_N/E`, `speed` | 小数点以下3桁 |
| `ekf.azimuth` | 小数点以下2桁 |
| `ekf.bias_accel/gyro` | 小数点以下6桁 |
| `sendTest()` のフィールド | `isInt`なら整数、`isFloat`なら有効数字7桁、それ以外(double)は有効数字15桁 |
