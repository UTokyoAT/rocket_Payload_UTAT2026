#pragma once
#include <Arduino.h>

class Actuator {
public:
    void begin();

    void setMotor(int speed);       // -255〜255（負=逆転）
    void deployParachute();         // ニクロム線 or サーボ
    void separate();                // ロケット分離機構

    void setBuzzer(bool on);
    void setLED(bool on);
    void beepPattern();             // 着地後の回収支援パターン
};
