#pragma once
#include "shared.h"
#include <GPS.h>

// XIAO ESP32S3: D7(GPIO44)にGPSモジュールのTXを、D6(GPIO43)にGPSモジュールのRXを接続
static const int GPS_RX_PIN = 44;  // D7
static const int GPS_TX_PIN = 43;  // D6

void taskGPS(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    GPS gps;
    gps.begin(GPS_RX_PIN, GPS_TX_PIN);

    for (;;) {
        gps.update();

        if (gps.isValid()) {
            if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
                s->latest.lat = gps.getLat();
                s->latest.lon = gps.getLon();
                xSemaphoreGive(s->mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
