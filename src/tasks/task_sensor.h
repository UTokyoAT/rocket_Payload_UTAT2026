#pragma once
#include "shared.h"
#include <Sensor.h>

void taskSensor(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    Sensor sensor;
    sensor.begin();

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        sensor.update();

        SensorData d;
        d.timestamp_ms = millis();
        d.alt   = sensor.getAltitude();
        d.roll  = sensor.getRoll();
        d.pitch = sensor.getPitch();
        d.yaw   = sensor.getYaw();

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            s->latest = d;
            xSemaphoreGive(s->mutex);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));  // 50Hz
    }
}
