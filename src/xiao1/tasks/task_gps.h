#pragma once
#include "shared.h"
#include <GPS.h>

// XIAO ESP32S3: D6(GPIO43)にGPSモジュールのTXを、D7(GPIO44)にGPSモジュールのRXを接続
// （回路図のネット名がD6=UART_RX、D7=UART_TXになっているため、それに合わせた割り当て）
static const int GPS_RX_PIN = 43;  // D6
static const int GPS_TX_PIN = 44;  // D7

void taskGPS(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    GPS gps;
    gps.begin(GPS_RX_PIN, GPS_TX_PIN);

    for (;;) {
        gps.update();

        bool valid = gps.isValid();
        // getLat()/getLon()より先に読むこと（呼ぶと内部の更新フラグが消費されるため）
        bool freshFix = gps.locationUpdated();
        int  satellites = gps.satellites();
        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            s->latest.gpsValid = valid;
            s->latest.gpsSatellites = satellites;
            if (valid) {
                s->latest.lat = gps.getLat();
                s->latest.lon = gps.getLon();
                if (freshFix) s->latest.gpsFixSeq++;
            }
            xSemaphoreGive(s->mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
