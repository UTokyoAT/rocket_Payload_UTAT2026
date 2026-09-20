# 無線データ収集サーバー

`無線サーバー設計要件書.md` の設計に基づく、Mosquitto / Telegraf / InfluxDB / Grafana の Docker Compose 構成。
ロボットと同一Wi-Fiネットワーク内での運用を前提とし、インターネット接続には依存しない。

## 構成

```
ESP32-S3 --(MQTT, QoS1)--> Mosquitto --(subscribe)--> Telegraf --(write)--> InfluxDB <-- Grafana
```

## 起動方法

1. `.env.example` を `.env` にコピーし、必要に応じて値を変更する
   ```
   cp .env.example .env
   ```
2. 起動
   ```
   docker compose up -d
   ```
3. 各サービスへのアクセス
   - Mosquitto: `tcp://<PCのIP>:1883` (ESP32からのpublish先)
   - InfluxDB UI: http://localhost:8086
   - Grafana: http://localhost:3000 (初期ログインは `.env` の `GRAFANA_ADMIN_USER` / `GRAFANA_ADMIN_PASSWORD`)
   - InfluxDBデータソースはGrafanaに自動プロビジョニング済み

`.env` の `INFLUXDB_INIT_*` は **InfluxDBの初回起動時にだけ** 使われる。一度起動した後に `INFLUXDB_INIT_ADMIN_TOKEN` などを変えるとTelegrafの認証が通らなくなる(`401 Unauthorized`)。
変えたい場合は、初期化してよいなら `docker compose down -v` でvolumeごと消してから起動し直す。データを残すなら `.env` を元の値に戻す。

## MQTTトピック(ファームウェアの `lib/Telemetry` と対応)

| トピック | 送信元API | QoS | measurement | 用途 |
|---|---|---|---|---|
| `rocket/telemetry` | `publishRecord()` | 1 | `rocket_telemetry` | 本番の統合レコード(10Hz)。`seq`(連番)と`uptime_ms`(起動からの経過時間)を含む |
| `rocket/test/<name>` | `sendTest()` | 0 | `rocket_test` (tag: `test_name`) | 切り分けテスト。送った数値フィールド(と`uptime_ms`)だけをそのまま保存 |
| `rocket/log` | `log()` | 0 | `rocket_log` (field: `value`) | テキスト1行のログ |

本番レコードは欠けたフィールド(センサー故障・GNSS未取得など)があっても、あるものだけ書き込まれる。

**時刻について**: ロボットは絶対時刻(GNSS等)を持たない。InfluxDBの`_time`はサーバーが受信した時刻で、リアルタイム表示にはそのまま使える。
ただし通信断から再送された分は受信時刻がずれるので、走行後の解析では次のフィールドを使う。

- `uptime_ms` … 時間軸(ロボット起動からの経過時間)
- `seq` … 欠落と重複の検出(QoS1は稀に同じレコードが2回届く。`seq`が同じなら重複)
- ロボットを再起動すると`uptime_ms`も`seq`も0から数え直す。別の走行かどうかは`_time`で区別する

## 動作確認(mosquitto_pubでダミーデータ送信)

本番レコード(`encoder`のキーは `right_rev` / `left_rev`):

```
docker compose exec mosquitto mosquitto_pub -t rocket/telemetry -q 1 -m '{
  "seq": 1,
  "uptime_ms": 12345,
  "pos": { "lat": 35.123456, "lon": 139.123456, "alt": 12.3 },
  "imu": { "ax": 0.01, "ay": -0.02, "az": 9.79, "gx": 0.1, "gy": 0.0, "gz": -0.1 },
  "baro": { "pressure_hpa": 1013.2, "alt_m": 12.1 },
  "encoder": { "right_rev": 34.55676, "left_rev": 42.44553 },
  "gnss": { "fix": 1, "hdop": 1.1, "sats": 9 },
  "EKF": { "position_N": 24.55, "position_E": 10.34, "speed": 0.75, "azimuth": 156, "bias_accel": 0.858685, "bias_gyro": 0.0374 }
}'
```

切り分けテストとログ:

```
docker compose exec mosquitto mosquitto_pub -t rocket/test/bmp280 -m '{"uptime_ms": 1000, "pressure_hpa": 1013.2, "alt_m": 12.1}'
docker compose exec mosquitto mosquitto_pub -t rocket/log -m '[1000ms] hello'
```

InfluxDB UI の Data Explorer で `rocket_telemetry` / `rocket_test` / `rocket_log` にデータが入っていることを確認できる。

## 停止方法

各コンテナは `restart: unless-stopped` のため、止めない限りPC/Docker再起動後も自動的に立ち上がり続ける。
将来的な常時稼働機(Raspberry Pi等)ではこれでよいが、普段の開発PCでは使わない時は停止しておく。

- 通常はこちらを使う: データを残したままコンテナだけ停止する
  ```
  docker compose stop
  ```
  再開するときは `docker compose up -d` または `docker compose start`

- コンテナ自体を削除する場合(named volumeは残るのでInfluxDB/Grafanaのデータは消えない)
  ```
  docker compose down
  ```

- データも含めて完全に初期化したい場合(要注意: 全データ消失)
  ```
  docker compose down -v
  ```

## 未決定事項(設計要件書 8章より、実装時に確定させること)

- MQTTのトピック名 (現在は仮で `rocket/telemetry` を使用。`telegraf/telegraf.conf` と ESP32側の送信先を合わせて変更する)
- MQTT認証方式(ユーザー名/パスワード、TLS)。現状 `mosquitto.conf` は `allow_anonymous true`
- InfluxDBのバケット構成・リテンションポリシー
- ESP32側バッファ容量設計(本サーバー構成の範囲外)

## ディレクトリ構成

```
ground/server/
├── docker-compose.yml
├── .env.example
├── mosquitto/config/mosquitto.conf
├── telegraf/telegraf.conf
└── grafana/provisioning/datasources/influxdb.yml
```
