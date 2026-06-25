#pragma once
#include "shared.h"
#include <Radio.h>

static const char* AP_SSID = "CanSat-AP";
static const char* AP_PASS = "cansat2026";

void taskWifi(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    Radio radio;
    radio.begin(AP_SSID, AP_PASS);

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        SensorData d;
        MissionState state;

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            d     = s->latest;
            state = s->state;
            xSemaphoreGive(s->mutex);
        }

        radio.broadcast(d, state);
        radio.cleanup();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));  // 20Hz
    }
}
