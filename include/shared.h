#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>

// FALLINGという名前はArduino.hの割り込みモード用マクロ(#define FALLING 0x02)と
// 衝突する（enum classでもマクロ展開はスコープを無視して行われるため）。DESCENDINGを使う。
enum class MissionState {
    STANDBY,
    ASCENDING,
    DESCENDING,
    SEPARATING,
    RUNNING,
    GOAL
};

struct SensorData {
    float alt   = 0.0f;
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
    double lat  = 0.0;
    double lon  = 0.0;
    uint32_t timestamp_ms = 0;
    bool gpsValid = false;         // taskGPSが書き込む。fix取得前はtaskNavigationがPID出力を0に固定する
    float pidOutput      = 0.0f;  // taskNavigationが計算する誘導PIDの旋回量。taskSpiLinkがXIAO2へ転送する
    float destinationYaw = 0.0f;  // taskNavigationが計算する目的地への方位角 [deg]（磁北基準）
};

struct Shared {
    SemaphoreHandle_t mutex;
    SensorData latest;
    MissionState state = MissionState::STANDBY;
};
