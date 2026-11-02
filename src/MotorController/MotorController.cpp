#include "MotorController.h"

static const int PWM_MAX = 255;  // Actuator の analogWrite 分解能

void MotorController::begin(Actuator& actuator, Encoder& encoder, const MotorControlConfig& config) {
    _actuator = &actuator;
    _encoder  = &encoder;
    cfg = config;

    v_max = cfg.eta_load * (TWO_PI * cfg.rpm_noload / 60.0) * cfg.r;
    // フィードフォワード未設定なら「デューティ100%で無負荷周速」とみなす
    Kff = (cfg.Kff > 0) ? cfg.Kff : 1.0 / ((TWO_PI * cfg.rpm_noload / 60.0) * cfg.r);

    stop();
}

void MotorController::setGoal(double N, double E) {
    N_goal = N;
    E_goal = E;
    I_head = 0;
    sat_prev = false;
    omega_cmd = 0;
    stopCount = 0;
    stopped = false;
}

void MotorController::stop() {
    stopped = true;
    omega_cmd = 0;
    I_head = 0;
    resetWheels();
    applyOutputs();
}

// ---------------------------------------------------------------------------
// 外側：方位PID → 速度計画 → 差動分配・飽和 → 急停止判定（要件書2.4.2〜2.4.4節）
// ---------------------------------------------------------------------------
void MotorController::updateGuidance(const KalmanFilter& ekf, double omega_meas) {
    if (!_actuator || stopped || !ekf.isInitialized()) {
        setWheelSpeed(0, 0);
        return;
    }

    const double dN = N_goal - ekf.getPositionN();
    const double dE = E_goal - ekf.getPositionE();
    d_hat    = hypot(dN, dE);
    psi_goal = atan2(dE, dN);
    e = KalmanFilter::wrapToPi(psi_goal - ekf.getAzimuth());  // 真北をまたいでも跳ねないよう必ず wrap

    // 急停止：d̂ ≤ d_stop が連続 N_stop ループ成立したら、このループで両輪 0
    stopCount = (d_hat <= cfg.d_stop) ? stopCount + 1 : 0;
    if (stopCount >= cfg.N_stop) {
        stop();
        return;
    }

    // 方位PID。D項は誤差の差分ではなくバイアス補正済みジャイロから取る
    const double omega_rate = omega_meas - ekf.getGyroBias();

    // 積分（アンチワインドアップ：前回飽和中は、飽和を解消する向きの誤差のときだけ積分）
    if (cfg.Ki != 0) {
        if (!sat_prev || (e > 0) != (omega_cmd > 0)) {
            I_head += e * cfg.dt_outer;
        }
        I_head = constrain(I_head, -cfg.I_max, cfg.I_max);
    }

    const double omega_raw = cfg.Kp * e + cfg.Ki * I_head - cfg.Kd * omega_rate;
    omega_cmd = constrain(omega_raw, -cfg.omega_cmd_max, cfg.omega_cmd_max);
    sat_prev  = (omega_cmd != omega_raw);

    // 速度計画：向きが大きくずれているときは前進を弱めて旋回を優先
    const double half_L = cfg.L / 2.0;
    double v_base = cfg.v_cruise * max(0.0, cos(e));
    // 飽和処理：車輪が v_max を超えるなら v_base 側を削る（後退はしない。その場旋回は許容）
    v_base = min(v_base, v_max - half_L * fabs(omega_cmd));
    v_base = max(v_base, 0.0);

    // 差動分配（ω_cmd > 0 で右旋回 → 右輪が遅い）
    setWheelSpeed(v_base - half_L * omega_cmd,
                  v_base + half_L * omega_cmd);
}

void MotorController::setWheelSpeed(double v_R_cmd, double v_L_cmd) {
    wheelR.v_cmd = constrain(v_R_cmd, -v_max, v_max);
    wheelL.v_cmd = constrain(v_L_cmd, -v_max, v_max);
}

// ---------------------------------------------------------------------------
// 内側：車輪速度PI（フィードフォワード＋デッドバンド補償＋アンチワインドアップ）
// ---------------------------------------------------------------------------
void MotorController::update() {
    if (!_actuator) return;

    const unsigned long now = micros();
    const unsigned long elapsedUs = now - lastInnerUs;
    if (elapsedUs < (unsigned long)(cfg.dt_inner * 1e6)) return;
    lastInnerUs = now;
    const double dt = elapsedUs * 1e-6;  // 実際の経過時間で速度・積分を計算する

    const double angleR = wheelAngleR();
    const double angleL = wheelAngleL();
    wheelR.v_enc = (angleR - wheelR.lastAngle) / dt * cfg.r;
    wheelL.v_enc = (angleL - wheelL.lastAngle) / dt * cfg.r;
    wheelR.lastAngle = angleR;
    wheelL.lastAngle = angleL;

    runWheelPi(wheelR, dt);
    runWheelPi(wheelL, dt);
    applyOutputs();
}

void MotorController::runWheelPi(Wheel& w, double dt) {
    // 指令ほぼ 0 なら止める（デッドバンド補償や積分でじわじわ回り続けないように）
    if (fabs(w.v_cmd) < cfg.v_eps) {
        w.I = 0;
        w.u = 0;
        return;
    }

    const double e_v = w.v_cmd - w.v_enc;
    const double sgn = (w.v_cmd > 0) ? 1.0 : -1.0;
    const double u_ff = Kff * w.v_cmd + sgn * cfg.u_dz;

    // アンチワインドアップ：前回デューティが張り付いていて、さらに同じ向きに押す誤差なら積分しない
    const bool saturated = fabs(w.u) >= 1.0 && (e_v > 0) == (w.u > 0);
    if (!saturated && cfg.Ki_w > 0) {
        w.I += e_v * dt;
        const double I_lim = 1.0 / cfg.Ki_w;  // 積分項単体でデューティ ±100% まで
        w.I = constrain(w.I, -I_lim, I_lim);
    }

    w.u = constrain(u_ff + cfg.Kp_w * e_v + cfg.Ki_w * w.I, -1.0, 1.0);
}

void MotorController::applyOutputs() {
    if (!_actuator) return;
    _actuator->setMotorRight(cfg.motorSignR * (int)lround(wheelR.u * PWM_MAX));
    _actuator->setMotorLeft (cfg.motorSignL * (int)lround(wheelL.u * PWM_MAX));
}

void MotorController::resetWheels() {
    wheelR.v_cmd = wheelL.v_cmd = 0;
    wheelR.I = wheelL.I = 0;
    wheelR.u = wheelL.u = 0;
    if (_encoder) {
        wheelR.lastAngle = wheelAngleR();
        wheelL.lastAngle = wheelAngleL();
    }
    lastInnerUs = micros();
}

double MotorController::wheelAngleR() { return cfg.encSignR * _encoder->getRightRevolutions(); }
double MotorController::wheelAngleL() { return cfg.encSignL * _encoder->getLeftRevolutions(); }
