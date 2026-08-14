#pragma once
#include <Arduino.h>

class Sensor {
public:
    bool begin();
    void update();          // 100Hz目安で呼ぶ（IMU・気圧・地磁気読み取り＋姿勢フィルタ）

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
    float getRoll();        // [deg] ジャイロ積分+加速度の相補フィルタによる重力ベクトルから算出
    float getPitch();       // [deg] 同上
    float getYaw();         // [deg] -180〜180、北=0（roll/pitchでチルト補正した地磁気から算出）
    float getAccelMag();    // 合成加速度 [m/s^2]（衝撃検知用）

    // BMM350（地磁気）生値 [uT]。ハードアイアン較正未実施のため、水平回転させても
    // 原点を中心とした円を描かない（=一定方向にオフセットしている）場合はyawが
    // 全周ではなく一部の角度範囲にしか動かなくなる。診断用に生値を確認するためのgetter。
    float getMagX();
    float getMagY();
    float getMagZ();

    bool isBmp280Ready();
    bool isMpu6050Ready();
    bool isBmm350Ready();
};
