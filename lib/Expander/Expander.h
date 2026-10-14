#pragma once
#include <Arduino.h>

class Expander {
public:
    bool setup();

    bool enableOutput(int pin);  // 使うピンごとに呼び出してOUTPUT+LOWに設定する

    bool set_high(int pin);
    bool set_low(int pin);

    bool reset(); // 全ピンを一旦lowへ

private:
    static const int NUM_PINS = 8;
    bool _expanderReady = false;
    bool _pinEnabled[NUM_PINS] = {false};

    void reconfigureEnabledPins();
};
