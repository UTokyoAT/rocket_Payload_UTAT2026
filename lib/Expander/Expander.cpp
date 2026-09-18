#include <Wire.h>
#include "Adafruit_MCP23X08.h"
#include "Expander.h"

// MCP23008 I/Oエキスパンダ制御
// Connect pin #1 of the expander to Analog 5 (i2c clock)
// Connect pin #2 of the expander to Analog 4 (i2c data)
// Connect pins #3, 4 and 5 of the expander to ground (address selection)
// Connect pin #6 of the expander to 5V (power)
// Connect pin #18 (RESET, active LOW) to a microcontroller output pin via 10kΩ
// Connect pin #9 of the expander to ground (common ground)

static Adafruit_MCP23X08 mcp;

bool Expander::setup(int resetPin) {
    _resetPin = resetPin;
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, HIGH);  // 平常時はHIGH（リセット解除）

    _expanderReady = mcp.begin_I2C();
    if (!_expanderReady) {
        Serial.println("[Expander] MCP23008 not found");
        return false;
    }

    return true;
}

bool Expander::enableOutput(int pin) {
    if (!_expanderReady || pin < 0 || pin >= NUM_PINS) {
        return false;
    }
    mcp.pinMode(pin, OUTPUT);
    mcp.digitalWrite(pin, LOW);
    _pinEnabled[pin] = true;
    return true;
}

bool Expander::set_high(int pin) {
    if (!_expanderReady || pin < 0 || pin >= NUM_PINS) {
        return false;
    }
    if (!_pinEnabled[pin]) {
        Serial.printf("[Expander] pin %d is not enabled as output\n", pin);
        return false;
    }
    mcp.digitalWrite(pin, HIGH);
    return true;
}

bool Expander::set_low(int pin) {
    if (!_expanderReady || pin < 0 || pin >= NUM_PINS) {
        return false;
    }
    if (!_pinEnabled[pin]) {
        Serial.printf("[Expander] pin %d is not enabled as output\n", pin);
        return false;
    }
    mcp.digitalWrite(pin, LOW);
    return true;
}

bool Expander::reset() {
    if (!_expanderReady) {
        return false;
    }

    digitalWrite(_resetPin, LOW);
    delayMicroseconds(10);
    digitalWrite(_resetPin, HIGH);
    delay(1);  // 10kΩ経由のRC遅延を見込んで復帰を待つ

    // ハードリセットでレジスタが初期値（全ピンINPUT）に戻るため、有効化済みピンだけ再設定する
    reconfigureEnabledPins();

    return true;
}

void Expander::reconfigureEnabledPins() {
    for (int pin = 0; pin < NUM_PINS; pin++) {
        if (_pinEnabled[pin]) {
            mcp.pinMode(pin, OUTPUT);
            mcp.digitalWrite(pin, LOW);
        }
    }
}
