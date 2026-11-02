#pragma once
#include <Arduino.h>

class Encoder {
public:
    void begin();

    // 取り付け向きの設定。前進で角度が増えるなら +1、減るなら -1（それ以外の値は +1 として扱う）
    void setDirection(int signRight, int signLeft);

    // 累積の車輪回転角。名前は Revolutions だが戻り値は rad
    double getRightRevolutions();
    double getLeftRevolutions();

    // 前回呼び出しからの回転角の差分 [rad]（EKF設計 1.5節の θ_wheel(k) − θ_wheel(k−1)）。
    // 制御周期ごとに1回呼ぶこと。
    double getRightDeltaAngle();
    double getLeftDeltaAngle();

    double rightRevolutions = 0;
    double leftRevolutions = 0;

private:
    double rightLastAngle = 0;  // 前回 getRightDeltaAngle 呼び出し時の回転角 [rad]
    double leftLastAngle = 0;   // 前回 getLeftDeltaAngle 呼び出し時の回転角 [rad]
    int rightSign = +1, leftSign = +1;

    static void updateRight();
    static void updateLeft();

    static volatile long rightEncoderCount;
    static volatile uint8_t rightLastState;
    static volatile long leftEncoderCount;
    static volatile uint8_t leftLastState;
};