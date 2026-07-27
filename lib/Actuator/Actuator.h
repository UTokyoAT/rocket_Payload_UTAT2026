#pragma once
#include <Arduino.h>

// 左右2輪駆動。XIAO2側。TB6612FNGを標準の3ピン/モーター＋共有STBY方式で制御する
// （AIN1・AIN2で回転方向、PWMAで速度を指定。BIN1・BIN2・PWMBはもう片方のモーター。
//   STBYはXIAO2側でGPIOが余っているためソフト制御する）。
class Actuator {
public:
    void begin();

    void setMotorLeft(int speed);   // -255〜255（負=逆転）
    void setMotorRight(int speed);  // -255〜255（負=逆転）
};
