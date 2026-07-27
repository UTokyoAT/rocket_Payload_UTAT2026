#include <Arduino.h>
#include <Sensor.h>
#include <DebugLog.h>

// BMP280（気圧センサー）の動作確認
// PlatformIO で env:test-bmp280 を選択して書き込む
// デバッグ出力はWiFi経由。CanSat-AP（パスワード: cansat2026）に接続して
// http://192.168.4.1 を開くか、GET /log をポーリングする（USBシリアル不要）。

static Sensor sensor;
static DebugLog debug;

void setup() {
    Serial.begin(115200);
    debug.begin();

    debug.printf("[TEST] BMP280 check starting...");
    sensor.begin();
    if (!sensor.isBmp280Ready()) {
        debug.printf("[TEST] BMP280 not detected. Check wiring (I2C: 0x76/0x77).");
    } else {
        // begin()時に地上気圧がキャリブレーションされている（Sensor.cppのコメント参照）。
        // alt はこの基準からの相対高度になるため、起動直後は0m付近になるはず。
        debug.printf("[TEST] Ground level calibrated: %.2fhPa", sensor.getGroundLevelHpa());
    }
}

void loop() {
    sensor.update();

    debug.printf("pressure=%.2fhPa  temp=%.1fC  alt(relative)=%.2fm",
                  sensor.getPressure(), sensor.getTemperature(), sensor.getAltitude());

    debug.poll();
    delay(500);
}
