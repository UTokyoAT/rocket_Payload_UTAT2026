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

        radio.setData(d, state);
        radio.poll();

        // 地上局からの手動操作コマンドをSharedへ書き戻す（taskSpiLinkがXIAO2へ転送する）
        int16_t left  = radio.getMotorCommandLeft();
        int16_t right = radio.getMotorCommandRight();
        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            s->manualMotorLeft  = left;
            s->manualMotorRight = right;
            xSemaphoreGive(s->mutex);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));  // 20Hz
    }
}
