#pragma once
#include <Arduino.h>

class Sensor {
public:
    bool begin();
    void update();          // 100Hz目安で呼ぶ（IMU・気圧読み取り）

    // BMP280（気圧）生値
    float getPressure();    // [hPa]

    // MPU6050（6軸）生値
    float getAccelX();      // [m/s^2]
    float getAccelY();
    float getAccelZ();
    float getGyroX();       // [rad/s]
    float getGyroY();
    float getGyroZ();

    float getAltitude();    // [m] begin()時にキャリブレーションした地上気圧からの相対高度
    float getGroundLevelHpa();  // begin()でキャリブレーションされた基準気圧 [hPa]（診断用）

    bool isBmp280Ready();
    bool isMpu6050Ready();
};
