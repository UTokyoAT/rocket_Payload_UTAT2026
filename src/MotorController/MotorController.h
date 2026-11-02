#pragma once
#include <Arduino.h>
#include "Actuator.h"
#include "Encoder.h"
#include "KarmanFliter.h"

// CanSat 2輪誘導のモーター駆動制御
//
//   [外側] 方位PID   … EKF推定値 → 旋回角速度指令 ω_cmd → 左右輪の周速指令 v_R_cmd, v_L_cmd
//                       EKFと同じ周期（10 Hz）で updateGuidance() を呼ぶ
//   [内側] 車輪速度PI … 周速指令とエンコーダ実測から PWMデューティを決めて Actuator に出す
//                       loop() のたびに update() を呼ぶ（内部で dt_inner ごとに実行）
//
// 使い方（main）:
//   motor.begin(actuator, encoder);               // actuator・encoder は begin 済みのもの
//   motor.setGoal(N_goal, E_goal);                // P_start 原点のローカル座標 [m]
//   loop() {
//     if (EKF周期) {
//       ekf.ekf_update(a, omega, v_R, v_L, fix);
//       motor.updateGuidance(ekf, omega);         // omega は EKF に渡したのと同じジャイロ生値
//     }
//     motor.update();                             // 毎ループ呼ぶ
//   }
//
// 符号規約は EKF と同じ：方位は真北0・時計回り正、ω > 0 で右旋回（右輪が遅い）。
// エンコーダの角度差分は EKF 用の getXxxDeltaAngle() と干渉しないよう、
// このクラスでは getXxxRevolutions() の絶対角度から自前で差分を取る。

struct MotorControlConfig {
    // --- 機体・モーター（EkfConfig と同じ値にすること） ---
    double r          = 0.045;   // [m] タイヤ半径
    double L          = 0.20;    // [m] トレッド幅
    double rpm_noload = 71;      // [rpm] GA12-N20 無負荷回転数
    double eta_load   = 0.8;     // [-] 負荷時の回転数比（v_max = η・2π・rpm/60・r）

    // --- 外側：方位PID（要件書3.5節） ---
    double dt_outer      = 0.1;          // [s] updateGuidance() を呼ぶ周期（EKFの dt と同じ）
    double Kp            = 1.0;          // [1/s]
    double Ki            = 0.0;          // [1/s^2] 基本は 0
    double Kd            = 0.2;          // [s] ジャイロ角速度から取る
    double I_max         = 0.5;          // [rad·s] 積分クランプ
    double omega_cmd_max = 0.785398;     // [rad/s] 旋回角速度指令上限（45 deg/s、≤ 2・v_max/L）
    double v_cruise      = 0.2;          // [m/s] 巡航速度（≤ v_max）
    double d_stop        = 3.0;          // [m] 急停止判定距離
    int    N_stop        = 3;            // [回] 連続成立ループ数

    // --- 内側：車輪速度PI ---
    double dt_inner = 0.02;      // [s] 車輪速度PIの周期（50 Hz）
    double Kp_w     = 4.0;       // [duty/(m/s)]
    double Ki_w     = 30.0;      // [duty/(m·s^-1·s)]
    double Kff      = 0.0;       // [duty/(m/s)] フィードフォワード。0 なら 1/(無負荷周速) を使う（実測値に差し替える）
    double u_dz     = 0.08;      // [-] 回り出すデューティ（デッドバンド補償。実測値に差し替える）
    double v_eps    = 0.005;     // [m/s] これ未満の指令は 0 として扱い、モーターを止める

    // --- 取り付け向き（test-motor で確認して合わせる） ---
    int motorSignL = +1;         // 正のPWMで左輪が前進するなら +1、後退するなら -1
    int motorSignR = +1;
    int encSignL   = +1;         // 前進で左エンコーダの角度が増えるなら +1、減るなら -1
    int encSignR   = +1;
};

class MotorController {
public:
    void begin(Actuator& actuator, Encoder& encoder, const MotorControlConfig& config = MotorControlConfig());

    // ゴール位置 [m]（P_start 原点、北・東）。停止状態も解除する
    void setGoal(double N_goal, double E_goal);

    // 外側ループ：EKF推定値から左右輪の周速指令を決める（dt_outer ごとに呼ぶ）。
    //   omega_meas : ジャイロz軸生値 [rad/s]（時計回り正。EKFに渡した値と同じもの）
    // EKF未初期化・停止済みのときは指令を 0 にする。
    void updateGuidance(const KalmanFilter& ekf, double omega_meas);

    // 内側ループ：周速指令にエンコーダ実測を追従させ、PWMを出力する（毎ループ呼ぶ）
    void update();

    // 誘導を使わず周速指令を直接与える（キャリブレーション直進など）[m/s]
    void setWheelSpeed(double v_R_cmd, double v_L_cmd);

    // 急停止（両輪 0、積分リセット）。setGoal() まで updateGuidance() は 0 を出し続ける
    void stop();

    // --- 状態の取得（ログ用） ---
    bool   isStopped()       const { return stopped; }
    double getDistance()     const { return d_hat; }        // 推定ゴール距離 d̂ [m]
    double getGoalAzimuth()  const { return psi_goal; }     // ゴール方位 ψ_goal [rad]
    double getHeadingError() const { return e; }            // 方位誤差 e [rad]
    double getOmegaCmd()     const { return omega_cmd; }    // 旋回角速度指令 [rad/s]
    double getVMax()         const { return v_max; }        // 車輪の最大周速 [m/s]
    double getSpeedCmdR()    const { return wheelR.v_cmd; } // [m/s]
    double getSpeedCmdL()    const { return wheelL.v_cmd; }
    double getSpeedR()       const { return wheelR.v_enc; } // エンコーダ実測 [m/s]
    double getSpeedL()       const { return wheelL.v_enc; }
    double getDutyR()        const { return wheelR.u; }     // [-1, 1]
    double getDutyL()        const { return wheelL.u; }

private:
    struct Wheel {
        double v_cmd = 0;      // 周速指令 [m/s]
        double v_enc = 0;      // エンコーダ実測周速 [m/s]
        double I = 0;          // 速度誤差の積分 [m]
        double u = 0;          // 出力デューティ [-1, 1]
        double lastAngle = 0;  // 前回の車輪回転角 [rad]
    };

    double wheelAngleR();
    double wheelAngleL();
    void   runWheelPi(Wheel& w, double dt);
    void   applyOutputs();
    void   resetWheels();

    Actuator* _actuator = nullptr;
    Encoder*  _encoder  = nullptr;
    MotorControlConfig cfg;

    double v_max = 0;
    double Kff   = 0;

    // 外側
    double N_goal = 0, E_goal = 0;
    double I_head = 0;
    bool   sat_prev = false;
    double omega_cmd = 0;
    double d_hat = 0, psi_goal = 0, e = 0;
    int    stopCount = 0;
    bool   stopped = true;   // setGoal() まで動かない

    // 内側
    Wheel wheelR, wheelL;
    unsigned long lastInnerUs = 0;
};
