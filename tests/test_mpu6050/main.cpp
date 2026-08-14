#include <Arduino.h>
#include <Sensor.h>
#include <DebugLog.h>

// MPU6050（6軸IMU）の動作確認
// PlatformIO で env:test-mpu6050 を選択して書き込む
// デバッグ出力はWiFi経由。CanSat-AP（パスワード: cansat2026）に接続して
// http://192.168.4.1 を開くか、GET /log をポーリングする（USBシリアル不要）。

static Sensor sensor;
static DebugLog debug;

void setup() {
    Serial.begin(115200);
    debug.begin();

    debug.printf("[TEST] MPU6050 check starting...");
    sensor.begin();
    if (!sensor.isMpu6050Ready()) {
        debug.printf("[TEST] MPU6050 not detected. Check wiring (I2C: 0x68/0x69).");
    }
}

void loop() {
    sensor.update();

    debug.printf("[mpu ready=%s] accel[m/s^2] x=%.2f y=%.2f z=%.2f  gyro[deg/s] x=%.1f y=%.1f z=%.1f  roll=%.1f pitch=%.1f",
                 sensor.isMpu6050Ready() ? "OK" : "NG",
                 sensor.getAccelX(), sensor.getAccelY(), sensor.getAccelZ(),
                 sensor.getGyroX(), sensor.getGyroY(), sensor.getGyroZ(),
                 sensor.getRoll(), sensor.getPitch());

    debug.poll();
    delay(200);
}
