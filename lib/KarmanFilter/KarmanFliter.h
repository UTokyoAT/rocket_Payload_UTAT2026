#pragma once

// CanSat 2輪誘導用 拡張カルマンフィルタ（EKF）
// 設計：「EKF設計及びシミュレーション方法.md」1章・2.2節
//
// 状態ベクトル x = [p_N, p_E, v, θ, b_a, b_g]^T
//   p_N, p_E : 出発地点 P_start を原点とした北・東方向の座標 [m]
//   v        : 前後方向速度 [m/s]
//   θ        : 方位角。真北0・時計回り正 [rad]
//   b_a      : 加速度計バイアス [m/s^2]
//   b_g      : ジャイロバイアス [rad/s]
// 角速度 ω は上から見て時計回り（右旋回）を正とする。
// MPU6050をチップ面上向きで実装した場合、getGyroZ() の符号を反転して渡すこと。

// パラメータ（EKFに関係するもののみ）
struct EkfConfig {
    // 3.1 周期
    double dt = 0.1;                    // [s] 予測・エンコーダ更新の周期

    // 3.2 IMU（MPU6050）
    double sigma_a    = 0.26;           // [m/s^2] 加速度計白色雑音
    double sigma_g    = 0.000872665;    // [rad/s] ジャイロ白色雑音（0.05 deg/s）
    double sigma_rw_a = 0.001;          // [m/s^2/√s] 加速度計バイアス ランダムウォーク
    double sigma_rw_g = 0.000174533;    // [rad/s/√s] ジャイロバイアス ランダムウォーク（0.01 deg/s）

    // 3.2 エンコーダ（AS5600、左右別個体）
    double sigma_theta_R = 0.00872665;  // [rad] 右輪 角度読み取りノイズ（0.5 deg）
    double sigma_theta_L = 0.00872665;  // [rad] 左輪 角度読み取りノイズ（0.5 deg）

    // 3.2 GNSS（NEO-6M）
    double sigma_N     = 2.12;          // [m] 北方向 位置ノイズ
    double sigma_E     = 2.12;          // [m] 東方向 位置ノイズ
    double sigma_cog   = 0.00872665;    // [rad] COGノイズ（0.5 deg）
    double v_threshold = 0.2;           // [m/s] COG更新を有効とみなす最小SOG

    // 2.2 初期共分散 P0
    int    N_avg        = 10;           // [回] 測位の平均回数
    double alpha        = 3.0;          // [-] GNSS平均の安全係数（2〜4）
    double sigma_v0     = 0.01;         // [m/s] 静止中の速度の不確かさ
    double P0_theta_max = 9.8696044;    // [rad^2] D が極端に小さいときの P0(θ) 上限（π^2）

    // 3.3 機体
    double r = 0.045;                   // [m] タイヤ半径
    double L = 0.20;                    // [m] トレッド幅
};

// キャリブレーション（Phase 0〜2）の結果。EKF初期化の入力。
struct EkfCalibration {
    double lat_start, lon_start;        // [deg] P_start（Phase 0 の測位平均）。ローカル座標の原点になる
    double lat_init,  lon_init;         // [deg] P_init （Phase 2 の測位平均）
    double b_a0;                        // [m/s^2] 静止区間の加速度計平均
    double b_g0;                        // [rad/s] 静止区間のジャイロ平均
    double sigma_ba0;                   // [m/s^2] b_a0 の不確かさ（標準誤差×余裕係数など）
    double sigma_bg0;                   // [rad/s] b_g0 の不確かさ
};

// 1ステップ分の GNSS fix（位置・COG・SOG は同一fixで届く）
struct GnssFix {
    double lat;                         // [deg]
    double lon;                         // [deg]
    double cog_deg;                     // [deg] 真北基準・時計回り（NMEA RMC の Course Over Ground）
    double sog;                         // [m/s] Speed Over Ground（COG更新の可否判定に使う）
};

class KalmanFilter {
public:
    static constexpr int N = 6;         // 状態の次元
    enum { PN = 0, PE, V, THETA, BA, BG };

    // EKFの初期化（1.4〜1.7節の定数行列構成 + 2.2節の x0, P0 算出）。
    // Q = B・Q_w・B^T はここで一度だけ計算する。
    void ekf_setup(const EkfConfig& config, const EkfCalibration& cal);

    // EKFの更新（1.8節の順序：予測 → エンコーダ更新 → GNSS位置更新 → COG更新）。
    //   a_meas     : 加速度計の前後軸生値 [m/s^2]
    //   omega_meas : ジャイロのz軸生値 [rad/s]（時計回り正）
    //   v_R, v_L   : 左右輪の周速 [m/s]（Encoder の角度差分から wheelSpeed() で求める）
    //   fix        : このステップで新しいGNSS fixが届いていればそのポインタ、なければ nullptr
    void ekf_update(double a_meas, double omega_meas, double v_R, double v_L,
                    const GnssFix* fix = nullptr);

    // --- 個別ステップ（ekf_update から呼ばれる。単体でも使える） ---
    void predict(double a_meas, double omega_meas);
    void updateEncoder(double v_R, double v_L, double omega_meas);
    void updateGnssPosition(double p_N_gnss, double p_E_gnss);
    void updateCog(double cog_rad);
    bool cogUpdateEnabled(double sog) const { return sog >= cfg.v_threshold; }

    // --- ユーティリティ ---
    // 緯度経度 → P_start 原点のローカル座標 [m]（1.6節）
    void latLonToLocal(double lat, double lon, double& p_N, double& p_E) const;
    // 1周期分の車輪回転角の差分 [rad] から周速 [m/s] を求める（1.5節、cfg.dt と cfg.r を使う）
    double wheelSpeed(double delta_angle) const { return delta_angle / cfg.dt * cfg.r; }
    // [-π, π) への折り返し（mod(a + π, 2π) − π）
    static double wrapToPi(double a);

    // --- 推定値の取得 ---
    double getPositionN() const { return x[PN]; }
    double getPositionE() const { return x[PE]; }
    double getSpeed()     const { return x[V]; }
    double getAzimuth()   const { return x[THETA]; }
    double getAccelBias() const { return x[BA]; }
    double getGyroBias()  const { return x[BG]; }
    const double (&getState() const)[N] { return x; }
    const double (&getCovarianceMatrix() const)[N][N] { return P; }
    bool   isInitialized() const { return initialized; }

    // 初期化時に求めた値（2.2節、5.5節の出力用）
    double getInitialDistance() const { return D0; }   // D = |P_init − P_start| [m]
    double getInitialAzimuth()  const { return theta0; }

    // 直近のイノベーション y とその共分散 S の対角成分
    double y_enc[2]  = {0, 0}, S_enc[2]  = {0, 0};
    double y_gnss[2] = {0, 0}, S_gnss[2] = {0, 0};
    double y_cog = 0, S_cog = 0;
    bool   gnss_updated = false;        // 直近の ekf_update で GNSS位置更新を行ったか
    bool   cog_updated  = false;        // 直近の ekf_update で COG更新を行ったか

private:
    // 計算済みのイノベーション y から K を求めて x, P を更新する（M = 1 または 2）。
    // S は出力。
    template <int M>
    void applyInnovation(const double H[M][N], const double y[M], const double R[M][M], double S[M][M]);

    EkfConfig cfg;
    bool initialized = false;

    double x[N] = {0};
    double P[N][N] = {{0}};
    double Q[N][N] = {{0}};

    double H_enc[2][N];
    double R_enc[2][2];
    double H_gnss[2][N];
    double R_gnss[2][2];
    double H_cog[1][N];
    double R_cog[1][1];

    double lat0 = 0, lon0 = 0, cos_lat0 = 1;  // ローカル座標の原点（P_start）
    double D0 = 0, theta0 = 0;
};
