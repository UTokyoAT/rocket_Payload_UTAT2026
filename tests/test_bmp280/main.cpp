#include <Arduino.h>
#include <Sensor.h>

// BMP280（気圧センサー）の動作確認
// PlatformIO で env:test-bmp280 を選択して書き込む

static Sensor sensor;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("[TEST] BMP280 check starting...");
    sensor.begin();
    if (!sensor.isBmp280Ready()) {
        Serial.println("[TEST] BMP280 not detected. Check wiring (I2C: 0x76/0x77).");
    }
}

void loop() {
    sensor.update();

    Serial.print("pressure=");
    Serial.print(sensor.getPressure());
    Serial.print("hPa  temp=");
    Serial.print(sensor.getTemperature());
    Serial.print("C  alt=");
    Serial.print(sensor.getAltitude());
    Serial.println("m");

    delay(500);
}
