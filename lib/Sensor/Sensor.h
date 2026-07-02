#pragma once
#include <Arduino.h>

class Sensor {
public:
    bool begin();
    void update();          // 50Hz で呼ぶ（IMU・気圧読み取り＋フィルタ）

    // BMP280（気圧）生値
    float getPressure();    // [hPa]
    float getTemperature(); // [℃]

    // MPU6050（6軸）生値
    float getAccelX();      // [m/s^2]
    float getAccelY();
    float getAccelZ();
    float getGyroX();       // [deg/s]
    float getGyroY();
    float getGyroZ();

    float getAltitude();    // [m] begin()時にキャリブレーションした地上気圧からの相対高度
    float getGroundLevelHpa();  // begin()でキャリブレーションされた基準気圧 [hPa]（診断用）
    float getRoll();        // [deg]
    float getPitch();       // [deg]
    float getYaw();         // [deg] 0-360、北=0（BMM350のコンパス方位。TODO: roll/pitchによるチルト補正）
    float getAccelMag();    // 合成加速度 [m/s^2]（衝撃検知用）

    bool isBmp280Ready();
    bool isMpu6050Ready();
    bool isBmm350Ready();
};
