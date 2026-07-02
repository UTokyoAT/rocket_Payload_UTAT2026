#include "PID.h"

PID::PID(float kp, float ki, float kd, float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax) {}

void PID::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
}

float PID::update(float error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    _integral += error * dt;

    float derivative = (error - _prevError) / dt;
    _prevError = error;

    float output = _kp * error + _ki * _integral + _kd * derivative;

    // 出力飽和時は積分を巻き戻してワインドアップを防ぐ
    if (output > _outMax) {
        output = _outMax;
        _integral -= error * dt;
    } else if (output < _outMin) {
        output = _outMin;
        _integral -= error * dt;
    }

    return output;
}
