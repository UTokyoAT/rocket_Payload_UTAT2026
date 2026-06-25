#pragma once
#include <Arduino.h>
#include "shared.h"

class Radio {
public:
    void begin(const char* ssid, const char* password);
    void broadcast(const SensorData& data, MissionState state);
    void cleanup();
    int  clientCount();
};
