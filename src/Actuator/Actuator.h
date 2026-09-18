#pragma once
#include <Arduino.h>
#include "Expander.h"


// 左右2輪駆動。TB6612FNGを標準の3ピン/モーター＋共有STBY方式で制御する
// （AIN1・AIN2で回転方向、PWMAで速度を指定。BIN1・BIN2・PWMBはもう片方のモーター。
//   STBYはソフト制御する）。
class Actuator {
public:
    void begin(Expander& expander);  // expanderはsetup()済みのものを渡す

    void setMotorLeft(int speed);   // -255〜255（負=逆転）
    void setMotorRight(int speed);  // -255〜255（負=逆転）

private:
    Expander* _expander = nullptr;
};
