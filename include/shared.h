#pragma once
#include <Arduino.h>
#include <freertos/semphr.h>

enum class MissionState {
    STANDBY,
    ASCENDING,
    FALLING,
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
};

struct Shared {
    SemaphoreHandle_t mutex;
    SensorData latest;
    MissionState state = MissionState::STANDBY;
};
