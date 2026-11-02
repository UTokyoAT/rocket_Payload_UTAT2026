#pragma once
#include <Arduino.h>
#include "Actuator.h"
#include "KalmanFilter.h"

// CanSat 2輪誘導のモーター駆動制御（単純な方位PID → 左右PWMの差動）
//
//   基本は両輪ともデューティ duty_max（全速）で走る。
//   方位誤差 θe = wrap(θ̂ − ψ_goal)  … 正 = 機体がゴール方位より時計回りにずれている
//   から操作量  u = Kp・θe + Ki・∫θe dt + Kd・ω  を求め、
//       u > 0（時計回りにずれている）→ 左輪のデューティを u だけ下げて左に曲がる
//       u < 0（反時計回りにずれている）→ 右輪のデューティを |u| だけ下げて右に曲がる
//   車輪速度のフィードバックは行わない（エンコーダは EKF の観測にだけ使う）。
//
// 符号規約：方位は真北0・時計回り正。omega は時計回り正 [rad/s]（D項に使う）。
// dθe/dt ≈ ω（ゴール方位の変化は遅い）なので、D項は誤差の差分ではなくジャイロから取る。

struct MotorControlConfig {
    double dt       = 0.1;    // [s] updateGuidance() を呼ぶ周期（EKF の dt と同じ）
    double duty_max = 1.0;    // [-] 基本デューティ（直進時の左右の値、0〜1）
    double duty_min = 0.0;    // [-] 下げる側の下限。負にすると逆転し、その場旋回できる（−1 まで）
    double Kp    = 0.5;       // [1/rad]   θe 1 rad あたりの下げ幅
    double Ki    = 0.0;       // [1/(rad·s)]
    double Kd    = 0.1;       // [s/rad]   ジャイロ角速度 1 rad/s あたりの下げ幅
    double I_max = 1.0;       // [rad·s]   積分クランプ
    double d_stop = 3.0;      // [m] 急停止判定距離
    int    N_stop = 3;        // [回] 連続成立ループ数
    int motorSignL = +1;      // 正のPWMで左輪が前進するなら +1、後退するなら -1
    int motorSignR = +1;
};

class MotorController {
public:
    void begin(Actuator& actuator, const MotorControlConfig& config = MotorControlConfig());

    void setGoal(double N_goal, double E_goal);             // P_start 原点 [m]。停止状態も解除
    void updateGuidance(const KalmanFilter& ekf, double omega_meas);
    void setDuty(double duty_L, double duty_R);             // 直接指定 [-1, 1]
    void stop();

    bool   isStopped()       const { return stopped; }
    double getDistance()     const { return d_hat; }
    double getGoalAzimuth()  const { return psi_goal; }
    double getHeadingError() const { return theta_e; }
    double getControl()      const { return u; }
    double getDutyL()        const { return dutyL; }
    double getDutyR()        const { return dutyR; }

private:
    void applyOutputs();

    Actuator* _actuator = nullptr;
    MotorControlConfig cfg;

    double N_goal = 0, E_goal = 0;
    double I = 0;
    bool   sat_prev = false;
    double u = 0;
    double d_hat = 0, psi_goal = 0, theta_e = 0;
    double dutyL = 0, dutyR = 0;
    int    stopCount = 0;
    bool   stopped = true;   // setGoal() まで動かない
};
