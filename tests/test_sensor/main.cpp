#include <Arduino.h>
#include <Sensor.h>
#include <GPS.h>
#include <Radio.h>
#include <Heading.h>
#include "spi_protocol.h"

// センサー統合動作確認テスト（XIAO1単体）。
// PlatformIOで env:test-sensor を選択してXIAO1へ書き込む。
//
// 目的: BMP280（気圧）・MPU6050（6軸IMU）・BMM350（地磁気）の3センサーを
// Sensorクラス経由ですべて動かし、その値をWiFiで地上局へ送れることを確認する。
// 本番はセンサー管理・WiFi送信はXIAO1(センサー)とXIAO2(WiFi中継)に分かれているが、
// このテストではXIAO1だけで完結させる（XIAO2・SPIは一切使わない）。
//
// 地上局側の設計に合わせるため、送信フレームは本番と同じ SpiFrameToXiao2
// （lib/Radio, ground/receiver.py, lib/Radio/dashboard.hが共有する契約）をそのまま使う。
// つまりXIAO2を経由しなくても ground/receiver.py やブラウザダッシュボード
// （http://192.168.4.1 ）はコード変更なしでこのテストの送信データを受信できる。
// GPSは3センサーには含まれないが、フレームのlat/lon・地上局の地図表示を
// 意味のある値で埋めるために合わせて動かす（未配線・未Fixのままでも0を送るだけで動作はする）。
//
// yaw（フレーム送信・ダッシュボード表示用）はBMM350ではなくHeading（ジャイロ+GPS courseの
// 相補フィルタ）を使う。実機ではBMM350の近くの磁気源が強すぎて生値(magX/Y)がほとんど動かず
// コンパスとして使えなかったため（lib/Heading/Heading.hのコメント参照）。
// 生の地磁気値はmagX/Y/Zとしてシリアルに出力しているので、較正の余地があるかは別途確認できる。
//
// XIAO ESP32S3: D6(GPIO43)にGPSモジュールのTXを、D7(GPIO44)にGPSモジュールのRXを接続
// （回路図のネット名がD6=UART_RX、D7=UART_TXになっているため、それに合わせた割り当て）
static const int GPS_RX_PIN = 43;  // D6
static const int GPS_TX_PIN = 44;  // D7

static Sensor sensor;
static GPS gps;
static Heading heading;
static Radio radio;

static uint32_t _lastPrintMs = 0;
static uint32_t _lastUpdateMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Serial.println("[TEST] Sensor + WiFi integration check starting...");

    sensor.begin();
    Serial.printf("[TEST] BMP280=%s  MPU6050=%s  BMM350=%s\n",
                  sensor.isBmp280Ready()  ? "OK" : "NG",
                  sensor.isMpu6050Ready() ? "OK" : "NG",
                  sensor.isBmm350Ready()  ? "OK" : "NG");

    gps.begin(GPS_RX_PIN, GPS_TX_PIN);

    // ジャイロのヨー軸バイアスを較正する。この間は機体を静止させておくこと。
    Serial.println("[TEST] Calibrating gyro yaw bias... keep the device still.");
    float biasSum = 0.0f;
    const int BIAS_SAMPLES = 200;
    for (int i = 0; i < BIAS_SAMPLES; i++) {
        sensor.update();
        biasSum += sensor.getGyroX();
        delay(5);
    }
    float gyroBias = biasSum / BIAS_SAMPLES;
    heading.begin(gyroBias);
    Serial.printf("[TEST] gyro yaw bias = %.3f deg/s\n", gyroBias);

    radio.begin("CanSat-AP", "cansat2026");
    Serial.printf("[TEST] Radio ready. Connect to CanSat-AP and open http://%s\n",
                  radio.getIP().toString().c_str());

    _lastUpdateMs = millis();
}

void loop() {
    gps.update();
    // getLat()/getLon()より前に呼ぶ（呼ぶと内部の更新フラグが消費される）
    bool gpsFixUpdated = gps.locationUpdated();
    sensor.update();

    uint32_t now = millis();
    float dt = (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    bool sensorsReady = sensor.isBmp280Ready() && sensor.isMpu6050Ready() && sensor.isBmm350Ready();
    bool gpsValid = gps.isValid();

    heading.update(sensor.getGyroX(), dt,
                    gpsFixUpdated, gps.isCourseValid(), gps.getCourse(), gps.getSpeedMps());

    SpiFrameToXiao2 out{};
    out.timestamp_ms = now;
    out.alt   = sensor.getAltitude();
    out.roll  = sensor.getRoll();
    out.pitch = sensor.getPitch();
    out.yaw   = heading.get();
    out.lat   = gpsValid ? static_cast<float>(gps.getLat()) : 0.0f;
    out.lon   = gpsValid ? static_cast<float>(gps.getLon()) : 0.0f;
    // このテストには誘導PID・目的地の概念がないため常に0を送る
    out.pid_output      = 0.0f;
    out.destination_yaw = 0.0f;
    // 3センサーがすべて揃っているかをNAVIGATE/SETTINGとして地上局ダッシュボードに表示する
    // （本番のミッションFSMは経由しないため、ここでは「センサー準備完了」の代替表示として使う）
    out.mission_state = static_cast<uint8_t>(sensorsReady ? MissionState::NAVIGATE : MissionState::SETTING);

    radio.setData(out);
    radio.poll();

    if (now - _lastPrintMs >= 500) {
        _lastPrintMs = now;
        Serial.printf("alt=%6.2fm  R=%6.1f P=%6.1f Y=%6.1f  mag[uT] x=%6.1f y=%6.1f z=%6.1f  GPS=%.6f,%.6f (fix=%s sats=%d)  sensors=%s\n",
                      out.alt, out.roll, out.pitch, out.yaw,
                      sensor.getMagX(), sensor.getMagY(), sensor.getMagZ(),
                      gps.getLat(), gps.getLon(), gpsValid ? "yes" : "no", gps.satellites(),
                      sensorsReady ? "OK" : "NG");
    }

    delay(10);  // 100Hz目安（Sensor::update()の姿勢フィルタに合わせる）
}
