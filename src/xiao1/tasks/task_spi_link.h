#pragma once
#include "shared.h"
#include "spi_protocol.h"
#include <SpiLinkMaster.h>

// XIAO2との通信担当（マスター側）。
// センサー値・地上局からの手動操作コマンドをXIAO2へ送り、
// XIAO2が実際に出力したモータ値を受け取ってSharedへ書き戻す（taskWifiがテレメトリ配信に使う）。
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
            out.timestamp_ms = d.timestamp_ms;
            out.alt   = d.alt;
            out.roll  = d.roll;
            out.pitch = d.pitch;
            out.yaw   = d.yaw;
            out.lat   = static_cast<float>(d.lat);
            out.lon   = static_cast<float>(d.lon);
            state = s->state;

            out.manual_motor_left  = s->manualMotorLeft;
            out.manual_motor_right = s->manualMotorRight;
            xSemaphoreGive(s->mutex);
        }
        out.mission_state   = static_cast<uint8_t>(state);
        out.manual_override = 0;  // TODO: 手動/自律の調停ロジック未実装。常に自律優先として送る

        SpiFrameFromXiao2 in = link.transfer(out);

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            s->latest.motorOutputLeft  = in.motor_output_left;
            s->latest.motorOutputRight = in.motor_output_right;
            xSemaphoreGive(s->mutex);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));  // 20Hz、taskWifiと同周期
    }
}
