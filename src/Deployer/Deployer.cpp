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
    
    _expander->set_high(PIN_ROCKET);
    delay(DELAY_ms);
    _expander->set_low(PIN_ROCKET);
}

void Deployer::deployParachute() {
    if (!_expander) return;
    
    _expander->set_high(PIN_PARACHUTE);
    delay(DELAY_ms);
    _expander->set_low(PIN_PARACHUTE);
}
