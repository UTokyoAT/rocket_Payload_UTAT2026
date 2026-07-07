#include <Arduino.h>
#include <GPS.h>

// GY-GPSV2-NEO6M（NEO-6M系GPSモジュール）のUART通信・測位動作確認
// PlatformIO で env:test-gps を選択して書き込む

// XIAO ESP32S3: D7(GPIO44)にGPSモジュールのTXを、D6(GPIO43)にGPSモジュールのRXを接続
static const int GPS_RX_PIN = 44;  // D7 - GPSモジュールのTXへ接続
static const int GPS_TX_PIN = 43;  // D6 - GPSモジュールのRXへ接続
static const uint32_t GPS_BAUD = 9600;

static GPS gps;
static uint32_t _lastPrintMs = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("[TEST] GPS (GY-GPSV2-NEO6M) UART check starting...");
    gps.begin(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void loop() {
    gps.update();

    uint32_t now = millis();
    if (now - _lastPrintMs >= 1000) {
        _lastPrintMs = now;

        if (gps.isValid()) {
            Serial.printf("lat=%.6f lon=%.6f alt=%.1fm sats=%d hdop=%.1f\n",
                          gps.getLat(), gps.getLon(), gps.getAltitude(),
                          gps.satellites(), gps.hdop());
        } else if (gps.charsProcessed() < 10) {
            Serial.println("[TEST] no data received at all... (wiring: D6->GPS RX, D7<-GPS TX / 電源を確認)");
        } else {
            Serial.printf("[TEST] waiting for fix... chars=%lu sats=%d hdop=%.1f failedChecksum=%lu\n",
                          (unsigned long)gps.charsProcessed(), gps.satellites(), gps.hdop(),
                          (unsigned long)gps.failedChecksumCount());
        }
    }
}
