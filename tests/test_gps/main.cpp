#include <Arduino.h>
#include <GPS.h>
#include <Telemetry.h>
#include "secrets.h"

// GY-GPSV2-NEO6M（NEO-6M系GPSモジュール）のUART通信・測位動作確認
// PlatformIO で env:test-gps を選択して書き込む
// 事前に include/secrets.example.h を include/secrets.h にコピーして値を設定し、
// ground/server で `docker compose up -d` しておく。
//
// 確認方法（InfluxDBのData Explorer / Grafana）:
//   rocket_test の test_name=gps  … 測位中の1秒ごとの lat / lon / alt / sats / hdop
//   rocket_log                    … 1秒ごとの状態ログ行（Serialにも出力）

// XIAO ESP32S3: D7(GPIO44)にGPSモジュールのTXを、D6(GPIO43)にGPSモジュールのRXを接続
static const int GPS_RX_PIN = 44;  // D7 - GPSモジュールのTXへ接続
static const int GPS_TX_PIN = 43;  // D6 - GPSモジュールのRXへ接続
static const uint32_t GPS_BAUD = 9600;

static GPS gps;
static Telemetry telemetry;
static uint32_t _lastPrintMs = 0;

void setup() {
    Serial.begin(115200);
    delay(500);  // USB CDC安定待ち

    Telemetry::Config cfg;
    cfg.ssid     = WIFI_SSID;
    cfg.password = WIFI_PASSWORD;
    cfg.mqttHost = MQTT_HOST;
    telemetry.begin(cfg);

    telemetry.log("[TEST] GPS (GY-GPSV2-NEO6M) UART check starting...");
    gps.begin(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void loop() {
    gps.update();

    uint32_t now = millis();
    if (now - _lastPrintMs >= 1000) {
        _lastPrintMs = now;

        if (gps.isValid()) {
            auto staticLatLon = gps.getStaticLatLon(10);
            telemetry.log("static lat=%.6f static lon=%.6f", staticLatLon[0], staticLatLon[1]);
            telemetry.log("lat=%.6f lon=%.6f alt=%.1fm sats=%d hdop=%.1f",
                          gps.getLat(), gps.getLon(), gps.getAltitude(),
                          gps.satellites(), gps.hdop());
            telemetry.sendTest("gps", {{"lat", gps.getLat()},
                                       {"lon", gps.getLon()},
                                       {"alt_m", gps.getAltitude()},
                                       {"sats", gps.satellites()},
                                       {"hdop", gps.hdop()},
                                       {"static_lat", staticLatLon[0]},
                                       {"static_lon", staticLatLon[1]}});
        } else if (gps.charsProcessed() < 10) {
            telemetry.log("[TEST] no data received at all... (wiring: D6->GPS RX, D7<-GPS TX / 電源を確認)");
        } else {
            telemetry.log("[TEST] waiting for fix... chars=%lu sats=%d hdop=%.1f failedChecksum=%lu",
                          (unsigned long)gps.charsProcessed(), gps.satellites(), gps.hdop(),
                          (unsigned long)gps.failedChecksumCount());
        }
    }
}
