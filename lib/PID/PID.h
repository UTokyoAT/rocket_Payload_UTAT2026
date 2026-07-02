#pragma once

// センサー非依存の汎用PIDコントローラ。角度の正規化など単位固有の処理は呼び出し側で行う。
class PID {
public:
    PID(float kp, float ki, float kd, float outMin, float outMax);

    void reset();
    float update(float error, float dt);  // dt [s]

private:
    float _kp, _ki, _kd;
    float _outMin, _outMax;
    float _integral = 0.0f;
    float _prevError = 0.0f;
};
