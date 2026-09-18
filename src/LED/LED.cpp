#include "LED.h"

// LEDのためのエクスパンダGPIO番号
static const int PIN_LED = 7;

void LED::begin(Expander& expander) {
    _expander = &expander;

    _expander->enableOutput(PIN_LED);
}

void LED::on() {
    if (!_expander || _isLEDon) return;
    _expander->set_high(PIN_LED);
    _isLEDon = true;
}

void LED::off() {
    if (!_expander || !_isLEDon) return;
    _expander->set_low(PIN_LED);
    _isLEDon = false;
}