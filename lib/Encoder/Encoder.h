#pragma once
#include <Arduino.h>

class Encoder {
public:
    void begin();

    float getRightRevolutions();
    float getLeftRevolutions();

    float rightRevolutions = 0;
    float leftRevolutions = 0;

private:
    static void updateRight();
    static void updateLeft();

    static volatile long rightEncoderCount;
    static volatile uint8_t rightLastState;
    static volatile long leftEncoderCount;
    static volatile uint8_t leftLastState;
};