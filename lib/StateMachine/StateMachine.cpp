#include "StateMachine.h"

// TODO: しきい値は実測値で調整する
static const float LAUNCH_ALT_M      = 10.0f;   // 上昇検知高度
static const float DESCENT_ALT_M     = 5.0f;    // 降下検知高度
static const float IMPACT_ACCEL_MS2  = 30.0f;   // 衝撃加速度
static const float LAND_ALT_M        = 3.0f;    // 着地判定高度
static const uint32_t GOAL_TIMEOUT_MS = 60000;  // RUNNING→GOAL タイムアウト

MissionState StateMachine::update(const SensorData& d) {
    uint32_t elapsed = millis() - _stateEnteredAt;

    switch (_state) {
        case MissionState::STANDBY:
            if (d.alt > LAUNCH_ALT_M)
                transition(MissionState::ASCENDING);
            break;

        case MissionState::ASCENDING:
            if (d.alt < DESCENT_ALT_M || d.getAccelMag() > IMPACT_ACCEL_MS2)
                transition(MissionState::DESCENDING);
            break;

        case MissionState::DESCENDING:
            if (d.alt < LAND_ALT_M)
                transition(MissionState::SEPARATING);
            break;

        case MissionState::SEPARATING:
            // パラシュート分離後すぐ RUNNING へ（分離処理はタスク側で実行）
            transition(MissionState::RUNNING);
            break;

        case MissionState::RUNNING:
            if (elapsed > GOAL_TIMEOUT_MS)
                transition(MissionState::GOAL);
            break;

        case MissionState::GOAL:
            break;
    }

    return _state;
}

void StateMachine::transition(MissionState next) {
    Serial.printf("[SM] %d → %d\n", static_cast<int>(_state), static_cast<int>(next));
    _state = next;
    _stateEnteredAt = millis();
}
