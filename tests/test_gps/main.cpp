#include <Arduino.h>
#include <GPS.h>
#include <DebugLog.h>

// GY-GPSV2-NEO6M（NEO-6M系GPSモジュール）のUART通信・測位動作確認
// PlatformIO で env:test-gps を選択して書き込む
// デバッグ出力はWiFi経由。CanSat-AP（パスワード: cansat2026）に接続して
// http://192.168.4.1 を開くか、GET /log をポーリングする（USBシリアル不要）。

// XIAO ESP32S3: D6(GPIO43)にGPSモジュールのTXを、D7(GPIO44)にGPSモジュールのRXを接続
// （回路図のネット名がD6=UART_RX、D7=UART_TXになっているため、それに合わせた割り当て）
static const int GPS_RX_PIN = 43;  // D6 - GPSモジュールのTXへ接続
static const int GPS_TX_PIN = 44;  // D7 - GPSモジュールのRXへ接続
static const uint32_t GPS_BAUD = 9600;

static GPS gps;
static DebugLog debug;
static uint32_t _lastPrintMs = 0;

void setup() {
    Serial.begin(115200);
    debug.begin();

    debug.printf("[TEST] GPS (GY-GPSV2-NEO6M) UART check starting...");
    gps.begin(GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
}

void loop() {
    gps.update();
    debug.poll();

    uint32_t now = millis();
    if (now - _lastPrintMs >= 1000) {
        _lastPrintMs = now;

        if (gps.isValid()) {
            debug.printf("lat=%.6f lon=%.6f alt=%.1fm sats=%d hdop=%.1f",
                         gps.getLat(), gps.getLon(), gps.getAltitude(),
                         gps.satellites(), gps.hdop());
        } else if (gps.charsProcessed() < 10) {
            debug.printf("[TEST] no data received at all... (wiring: D6->GPS RX, D7<-GPS TX / 電源を確認)");
        } else {
            debug.printf("[TEST] waiting for fix... chars=%lu sats=%d hdop=%.1f failedChecksum=%lu",
                         (unsigned long)gps.charsProcessed(), gps.satellites(), gps.hdop(),
                         (unsigned long)gps.failedChecksumCount());
        }
    }
}
