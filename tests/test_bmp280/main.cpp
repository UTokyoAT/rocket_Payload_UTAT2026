#include <Arduino.h>
#include <Sensor.h>
#include <Telemetry.h>
#include "secrets.h"

// BMP280 の動作確認。PlatformIO で env:test-bmp280 を選択して書き込む。
// 事前に include/secrets.example.h を include/secrets.h にコピーして値を設定し、
// ground/server で `docker compose up -d` しておく。
//
// センサーは100Hzで読み、10Hz（本番の publishRecord と同じ周期）で地上局へ送る。
//
// 確認方法（InfluxDBのData Explorer / Grafana）:
//   rocket_test の test_name=bmp280 … 100msごとの以下のフィールド
//     ready              bmp2806050を検出できたか（1/0）
//     pressure           気圧の生値（hPa）
//     altitude           起動地点からの相対高度（m）。静止時は ≒ 0、手で持ち上げると +1m 程度変化
//   rocket_log                         … 1秒ごとの値の要約・接続状態

static Sensor    sensor;
static Telemetry telemetry;

static const uint32_t SAMPLE_PERIOD_MS = 10;    // 100Hz（Sensor::update()の想定周期）
static const uint32_t SEND_PERIOD_MS   = 100;   // 10Hz
static const uint32_t LOG_PERIOD_MS    = 1000;

static uint32_t _lastSendMs = 0;
static uint32_t _lastLogMs  = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.begin(cfg);

    sensor.begin();
    // begin()直後はまだ未接続なのでこのログはSerialにしか出ない
    telemetry.log("[TEST] BMP280 check starting... bmp=%s",
                  sensor.isBmp280Ready() ? "OK" : "NG");
}

void loop() {
    uint32_t start = millis();
    sensor.update();

    bool  ready = sensor.isBmp280Ready();
    float pressure = sensor.getPressure();
    float altitude = sensor.getAltitude();
    
    if (start - _lastSendMs >= SEND_PERIOD_MS) {
        _lastSendMs = start;
        if (ready) {
            telemetry.sendTest("bmp280", {{"ready", true},
                                           {"pressure", pressure}, {"altitude", altitude}});
        } else {
            // 未検出時は0埋めの値を送ると正常値と区別しにくいので ready だけ送る
            telemetry.sendTest("bmp280", {{"ready", false}});
        }
    }

    if (start - _lastLogMs >= LOG_PERIOD_MS) {
        _lastLogMs = start;
        if (ready) {
            telemetry.log("[bmp OK] pressure=%.3f, altitude=%.2f mqtt=%s",
                          pressure, altitude,
                          telemetry.isConnected() ? "yes" : "no");
        } else {
            telemetry.log("[bmp NG] BMP280 not detected. Check wiring (I2C: 0x76/0x77). mqtt=%s",
                          telemetry.isConnected() ? "yes" : "no");
        }
    }

    uint32_t elapsed = millis() - start;
    if (elapsed < SAMPLE_PERIOD_MS) delay(SAMPLE_PERIOD_MS - elapsed);
}
