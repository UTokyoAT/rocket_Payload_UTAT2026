#pragma once
#include "shared.h"
#include "spi_protocol.h"
#include <SpiLinkMaster.h>

// XIAO2との通信担当（マスター側）。
// センサー値・誘導PID出力をXIAO2へ送る。XIAO2はこれをモータへ反映しつつ、
// 同じフレームをWiFiテレメトリとして地上局へ中継する。
void taskSpiLink(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    SpiLinkMaster link;
    link.begin();

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        SpiFrameToXiao2 out{};
        MissionState state;

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            const SensorData& d = s->latest;
            out.timestamp_ms    = d.timestamp_ms;
            out.alt             = d.alt;
            out.roll            = d.roll;
            out.pitch           = d.pitch;
            out.yaw             = d.yaw;
            out.lat             = static_cast<float>(d.lat);
            out.lon             = static_cast<float>(d.lon);
            out.pid_output      = d.pidOutput;
            out.destination_yaw = d.destinationYaw;
            state = s->state;
            xSemaphoreGive(s->mutex);
        }
        out.mission_state = static_cast<uint8_t>(state);

        link.transfer(out);  // 応答フレームは現状未使用（SpiFrameFromXiao2参照）

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));  // 100Hz（taskNavigationのPID出力周期に合わせる）
    }
}
