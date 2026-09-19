#include <Arduino.h>
#include <Telemetry.h>
#include "secrets.h"

// Telemetryライブラリの疎通確認。PlatformIO で env:test-telemetry を選択して書き込む。
// 事前に include/secrets.example.h を include/secrets.h にコピーして値を設定し、
// ground/server で `docker compose up -d` しておく。
//
// 確認方法（InfluxDBのData Explorer / Grafana）:
//   rocket_test の test_name=smoke  … 200msごとの count / wave
//   rocket_log                       … 1秒ごとのログ行

static Telemetry telemetry;
static uint32_t  _count = 0;
static uint32_t  _lastLogMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.begin(cfg);

    telemetry.log("[TEST] telemetry smoke test starting");
}

void loop() {
    telemetry.sendTest("smoke", {{"count", _count++}, {"wave", sinf(millis() / 1000.0f)}});

    uint32_t now = millis();
    if (now - _lastLogMs >= 1000) {
        _lastLogMs = now;
        telemetry.log("[TEST] alive count=%lu connected=%s",
                      static_cast<unsigned long>(_count), telemetry.isConnected() ? "yes" : "no");
    }

    delay(200);
}
