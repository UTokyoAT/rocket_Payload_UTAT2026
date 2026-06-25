#pragma once
#include "shared.h"

class StateMachine {
public:
    MissionState update(const SensorData& data);
    MissionState getState() const { return _state; }

private:
    MissionState _state = MissionState::STANDBY;
    uint32_t     _stateEnteredAt = 0;   // 状態に入った時刻 [ms]

    void transition(MissionState next);
};
