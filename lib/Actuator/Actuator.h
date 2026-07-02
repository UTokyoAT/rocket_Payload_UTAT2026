#pragma once
#include <Arduino.h>

// 左右2輪駆動。TB6612FNGを2ピン/モーター方式で制御する
// （IN1・IN2のどちらか片方をPWM出力し、もう片方を0にすることで速度・方向を両方指定する。
//   PWMA/PWMB（イネーブル）・STBYはハード側で常時HIGHに固定配線しておく前提。
//   3ピン方式（IN1・IN2・PWMを別々に制御）に比べGPIOを2本節約できる —
//   XIAO ESP32S3はGPIOが11本しかなく、センサーI2C・GPS UARTと合わせると
//   3ピン方式では本数が足りないため）
class Actuator {
public:
    void begin();

    void setMotorLeft(int speed);   // -255〜255（負=逆転）
    void setMotorRight(int speed);  // -255〜255（負=逆転）
    void deployParachute();         // ニクロム線 or サーボ
    void separate();                // ロケット分離機構

    void setBuzzer(bool on);
    void setLED(bool on);
    void beepPattern();             // 着地後の回収支援パターン
};
