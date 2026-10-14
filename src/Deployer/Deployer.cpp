#include "Deployer.h"

// ロケットからの分離のためのニクロム線通電：GP1
// パラシュートの分離のためのニクロム線通電：GP0
static const int PIN_ROCKET    = 1;
static const int PIN_PARACHUTE = 0;
static const int DELAY_ms = 1000;

void Deployer::begin(Expander& expander) {
    _expander = &expander;

    _expander->enableOutput(PIN_ROCKET);
    _expander->enableOutput(PIN_PARACHUTE);
}

void Deployer::deployRocket() {
    if (!_expander) return;

    setRocket(true);
    delay(DELAY_ms);
    setRocket(false);
}

void Deployer::deployParachute() {
    if (!_expander) return;

    setParachute(true);
    delay(DELAY_ms);
    setParachute(false);
}

bool Deployer::setRocket(bool on) {
    if (!_expander) return false;
    return on ? _expander->set_high(PIN_ROCKET) : _expander->set_low(PIN_ROCKET);
}

bool Deployer::setParachute(bool on) {
    if (!_expander) return false;
    return on ? _expander->set_high(PIN_PARACHUTE) : _expander->set_low(PIN_PARACHUTE);
}
