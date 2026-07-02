#include <Arduino.h>
#include <Sensor.h>

// BMM350（地磁気センサー）の動作確認
// PlatformIO で env:test-bmm350 を選択して書き込む

static Sensor sensor;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("[TEST] BMM350 check starting...");
    sensor.begin();
    if (!sensor.isBmm350Ready()) {
        Serial.println("[TEST] BMM350 not detected. Check wiring (I2C: 0x14).");
    }
}

void loop() {
    sensor.update();

    Serial.print("compass=");
    Serial.print(sensor.getYaw());
    Serial.println("deg (0=North, 90=East, 180=South, 270=West)");

    delay(500);
}
