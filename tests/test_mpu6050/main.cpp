#include <Arduino.h>
#include <Sensor.h>

// MPU6050（6軸IMU）の動作確認
// PlatformIO で env:test-mpu6050 を選択して書き込む

static Sensor sensor;

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    Serial.println("[TEST] MPU6050 check starting...");
    sensor.begin();
    if (!sensor.isMpu6050Ready()) {
        Serial.println("[TEST] MPU6050 not detected. Check wiring (I2C: 0x68/0x69).");
    }
}

void loop() {
    sensor.update();

    Serial.print("[mpu ready=");
    Serial.print(sensor.isMpu6050Ready() ? "OK" : "NG");
    Serial.print("] ");

    Serial.print("accel[m/s^2] x=");
    Serial.print(sensor.getAccelX());
    Serial.print(" y=");
    Serial.print(sensor.getAccelY());
    Serial.print(" z=");
    Serial.print(sensor.getAccelZ());

    Serial.print("  gyro[deg/s] x=");
    Serial.print(sensor.getGyroX());
    Serial.print(" y=");
    Serial.print(sensor.getGyroY());
    Serial.print(" z=");
    Serial.print(sensor.getGyroZ());

    Serial.print("  roll=");
    Serial.print(sensor.getRoll());
    Serial.print(" pitch=");
    Serial.println(sensor.getPitch());

    delay(200);
}
