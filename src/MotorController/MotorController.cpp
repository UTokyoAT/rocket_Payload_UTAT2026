#include "MotorController.h"

static const int PWM_MAX = 255;  // Actuator の analogWrite 分解能

void MotorController::begin(Actuator& actuator, const MotorControlConfig& config) {
    _actuator = &actuator;
    cfg = config;
    stop();
}

void MotorController::setGoal(double N, double E) {
    N_goal = N;  E_goal = E;
    I = 0;  sat_prev = false;  u = 0;
    stopCount = 0;  stopped = false;
}

void MotorController::stop() {
    stopped = true;  I = 0;  u = 0;
    setDuty(0, 0);
}

void MotorController::updateGuidance(const KalmanFilter& ekf, double omega_meas) {
    if (!_actuator || stopped || !ekf.isInitialized()) { setDuty(0, 0); return; }

    const double dN = N_goal - ekf.getPositionN();
    const double dE = E_goal - ekf.getPositionE();
    d_hat    = hypot(dN, dE);
    psi_goal = atan2(dE, dN);                                       // コンパス規約
    theta_e  = KalmanFilter::wrapToPi(ekf.getAzimuth() - psi_goal);

    stopCount = (d_hat <= cfg.d_stop) ? stopCount + 1 : 0;
    if (stopCount >= cfg.N_stop) { stop(); return; }

    const double omega_rate = omega_meas - ekf.getGyroBias();   // 時計回り正
    const double u_lim = cfg.duty_max - cfg.duty_min;

    if (cfg.Ki != 0) {
        if (!sat_prev || (theta_e > 0) != (u > 0)) I += theta_e * cfg.dt;
        I = constrain(I, -cfg.I_max, cfg.I_max);
    }

    const double u_raw = cfg.Kp * theta_e + cfg.Ki * I + cfg.Kd * omega_rate;
    u = constrain(u_raw, -u_lim, u_lim);
    sat_prev = (u != u_raw);

    double dL = cfg.duty_max, dR = cfg.duty_max;
    if (u > 0) dL -= u;   // 左旋回
    else       dR += u;   // 右旋回
    setDuty(dL, dR);
}

void MotorController::setDuty(double duty_L, double duty_R) {
    dutyL = constrain(duty_L, -1.0, 1.0);
    dutyR = constrain(duty_R, -1.0, 1.0);
    applyOutputs();
}

void MotorController::applyOutputs() {
    if (!_actuator) return;
    _actuator->setMotorLeft (cfg.motorSignL * (int)lround(dutyL * PWM_MAX));
    _actuator->setMotorRight(cfg.motorSignR * (int)lround(dutyR * PWM_MAX));
}
