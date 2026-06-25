#pragma once
#include <Arduino.h>

class Sensor {
public:
    bool begin();
    void update();          // 50Hz で呼ぶ（IMU・気圧・地磁気読み取り＋フィルタ）

    float getAltitude();    // [m]
    float getRoll();        // [deg]
    float getPitch();       // [deg]
    float getYaw();         // [deg]
    float getAccelMag();    // 合成加速度 [m/s^2]（衝撃検知用）
};
