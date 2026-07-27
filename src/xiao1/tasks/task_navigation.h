#pragma once
#include "shared.h"
#include <PID.h>
#include <TinyGPSPlus.h>

// 目的地座標はShared::goalLat/goalLonを参照する（既定値はshared.hを参照。
// 地上局がXIAO2へGET /goal?lat=..&lon=..を送るとtaskSpiLink経由で上書きされる）。

// 東京の磁気偏角（西偏、約7.667度）。GPS方位(真北基準)とBMM350のyaw(磁北基準)を
// 比較する際に補正する。TODO: 実際の打ち上げ場所に合わせて変更する
static const float NAV_MAGNETIC_DECLINATION_DEG = 7.667f;

static float navNormalizeAngle(float deg) {
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// GPS方位（目標地点への方位）とセンサーyaw（磁北基準ヘディング）の誤差をPIDで
// 補正し、旋回量をSharedへ書き込む（taskSpiLinkがXIAO2へ転送する）。
// GPSのfixはtaskGPSが別周期（1Hz程度）で更新するため、ここでは毎回Shared.latest.lat/lonの
// 最新値を読むだけで、GPS自体の更新頻度には同期しない。
void taskNavigation(void* arg) {
    Shared* s = static_cast<Shared*>(arg);
    // TODO: 実機でゲイン調整。100Hz周期(dt=0.01s)前提
    PID headingPid(2.0f, 0.0f, 0.5f, -255.0f, 255.0f);

    TickType_t last = xTaskGetTickCount();
    uint32_t lastUpdateMs = millis();

    for (;;) {
        float yaw;
        double lat, lon;
        bool gpsValid;
        double goalLat, goalLon;

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            yaw      = s->latest.yaw;
            lat      = s->latest.lat;
            lon      = s->latest.lon;
            gpsValid = s->latest.gpsValid;
            goalLat  = s->goalLat;
            goalLon  = s->goalLon;
            xSemaphoreGive(s->mutex);
        }

        uint32_t now = millis();
        float dt = (now - lastUpdateMs) / 1000.0f;
        lastUpdateMs = now;

        float pidOutput = 0.0f;
        float destinationYaw = 0.0f;

        if (gpsValid) {
            destinationYaw = static_cast<float>(TinyGPSPlus::courseTo(lat, lon, goalLat, goalLon));
            float error = navNormalizeAngle(destinationYaw + NAV_MAGNETIC_DECLINATION_DEG - yaw);
            pidOutput = headingPid.update(error, dt);
        } else {
            headingPid.reset();  // fix取得前は積分を溜め込まない
        }

        if (xSemaphoreTake(s->mutex, pdMS_TO_TICKS(5))) {
            s->latest.pidOutput      = pidOutput;
            s->latest.destinationYaw = destinationYaw;
            xSemaphoreGive(s->mutex);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));  // 100Hz（taskSensorのyaw更新周期に合わせる）
    }
}
