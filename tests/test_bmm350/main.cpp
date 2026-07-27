#include <Arduino.h>
#include <Sensor.h>
#include <DebugLog.h>

// BMM350（地磁気センサー）の動作確認
// PlatformIO で env:test-bmm350 を選択して書き込む
// デバッグ出力はWiFi経由。CanSat-AP（パスワード: cansat2026）に接続して
// http://192.168.4.1 を開くか、GET /log をポーリングする（USBシリアル不要）。

static Sensor sensor;
static DebugLog debug;

void setup() {
    Serial.begin(115200);
    debug.begin();

    debug.printf("[TEST] BMM350 check starting...");
    sensor.begin();
    if (!sensor.isBmm350Ready()) {
        debug.printf("[TEST] BMM350 not detected. Check wiring (I2C: 0x14).");
    }
}

void loop() {
    sensor.update();

    debug.printf("compass=%.1fdeg (0=North, 90=East, 180=South, 270=West)", sensor.getYaw());

    debug.poll();
    delay(500);
}
