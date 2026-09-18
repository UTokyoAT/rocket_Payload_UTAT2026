#pragma once
#include <Arduino.h>
#include "Expander.h"

// エクスパンサ経由。

class LED {
public:
    void begin(Expander& expander);  // expanderはsetup()済みのものを渡す

    void on();
    void off();

private:
    Expander* _expander = nullptr;
    bool _isLEDon = false;
};