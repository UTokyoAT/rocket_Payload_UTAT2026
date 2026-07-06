#pragma once
#include "shared.h"
#include <GPS.h>

// TODO: 実際のピン番号に変更する
static const int GPS_RX_PIN = 16;
static const int GPS_TX_PIN = 17;

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
