#include "KarmanFliter.h"
#include <math.h>
#include <string.h>

namespace {
constexpr double EARTH_M_PER_DEG = 111320.0;  // [m/deg] 緯度1度あたりの距離（1.6節）
constexpr double DEG2RAD = M_PI / 180.0;

// イノベーション共分散 S の逆行列（観測次元 1, 2 用）
void invert(const double S[1][1], double Sinv[1][1]) {
    Sinv[0][0] = 1.0 / S[0][0];
}

void invert(const double S[2][2], double Sinv[2][2]) {
    const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    Sinv[0][0] =  S[1][1] / det;
    Sinv[0][1] = -S[0][1] / det;
    Sinv[1][0] = -S[1][0] / det;
    Sinv[1][1] =  S[0][0] / det;
}
}

// =============================================================================
// 初期化
// =============================================================================
void KalmanFilter::ekf_setup(const EkfConfig& config, const EkfCalibration& cal) {
    cfg = config;
    const double dt = cfg.dt;

    // --- プロセスノイズ（1.4節）Q = B・Q_w・B^T ---
    // B は対角ブロックのみなので結果は対角行列：
    // Q = diag(0, 0, σ_a²Δt², σ_g²Δt², σ_rw_a²Δt, σ_rw_g²Δt)
    const double B[N][4] = {
        {0,  0,  0,         0},
        {0,  0,  0,         0},
        {dt, 0,  0,         0},
        {0,  dt, 0,         0},
        {0,  0,  sqrt(dt),  0},
        {0,  0,  0,         sqrt(dt)},
    };
    const double Qw[4] = {
        cfg.sigma_a * cfg.sigma_a,
        cfg.sigma_g * cfg.sigma_g,
        cfg.sigma_rw_a * cfg.sigma_rw_a,
        cfg.sigma_rw_g * cfg.sigma_rw_g,
    };
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            double s = 0;
            for (int k = 0; k < 4; k++) s += B[i][k] * Qw[k] * B[j][k];
            Q[i][j] = s;
        }

    // --- エンコーダ（1.5節） ---
    memset(H_enc, 0, sizeof(H_enc));
    H_enc[0][V] = 1;  H_enc[0][BG] = -cfg.L / 2;
    H_enc[1][V] = 1;  H_enc[1][BG] = cfg.L / 2;
    memset(R_enc, 0, sizeof(R_enc));
    R_enc[0][0] = cfg.r * cfg.r * 2 * cfg.sigma_theta_R * cfg.sigma_theta_R / (dt * dt);
    R_enc[1][1] = cfg.r * cfg.r * 2 * cfg.sigma_theta_L * cfg.sigma_theta_L / (dt * dt);

    // --- GNSS位置（1.6節） ---
    memset(H_gnss, 0, sizeof(H_gnss));
    H_gnss[0][PN] = 1;
    H_gnss[1][PE] = 1;
    memset(R_gnss, 0, sizeof(R_gnss));
    R_gnss[0][0] = cfg.sigma_N * cfg.sigma_N;
    R_gnss[1][1] = cfg.sigma_E * cfg.sigma_E;

    // --- GNSS COG（1.7節） ---
    memset(H_cog, 0, sizeof(H_cog));
    H_cog[0][THETA] = 1;
    R_cog[0][0] = cfg.sigma_cog * cfg.sigma_cog;

    // --- ローカル座標の原点を P_start にする ---
    lat0 = cal.lat_start;
    lon0 = cal.lon_start;
    cos_lat0 = cos(lat0 * DEG2RAD);

    // --- 初期状態 x0（2.2節） ---
    double pN_init, pE_init;
    latLonToLocal(cal.lat_init, cal.lon_init, pN_init, pE_init);
    D0 = hypot(pN_init, pE_init);
    theta0 = atan2(pE_init, pN_init);   // コンパス規約：atan2(東, 北)

    x[PN]    = pN_init;
    x[PE]    = pE_init;
    x[V]     = 0;
    x[THETA] = theta0;
    x[BA]    = cal.b_a0;
    x[BG]    = cal.b_g0;

    // --- 初期共分散 P0（2.2節） ---
    const double sigma_p2 = (cfg.sigma_N * cfg.sigma_N + cfg.sigma_E * cfg.sigma_E) / 2;
    double P0_theta = (D0 > 0) ? cfg.alpha * 2 * sigma_p2 / (cfg.N_avg * D0 * D0) : cfg.P0_theta_max;
    if (P0_theta > cfg.P0_theta_max) P0_theta = cfg.P0_theta_max;

    memset(P, 0, sizeof(P));
    P[PN][PN]       = cfg.alpha * cfg.sigma_N * cfg.sigma_N / cfg.N_avg;
    P[PE][PE]       = cfg.alpha * cfg.sigma_E * cfg.sigma_E / cfg.N_avg;
    P[V][V]         = cfg.sigma_v0 * cfg.sigma_v0;
    P[THETA][THETA] = P0_theta;
    P[BA][BA]       = cal.sigma_ba0 * cal.sigma_ba0;
    P[BG][BG]       = cal.sigma_bg0 * cal.sigma_bg0;

    initialized = true;
}

// =============================================================================
// 更新（1.8節のスケジューリング）
// =============================================================================
void KalmanFilter::ekf_update(double a_meas, double omega_meas, double v_R, double v_L,
                              const GnssFix* fix) {
    gnss_updated = false;
    cog_updated  = false;
    if (!initialized) return;

    // 1. 予測ステップ（必ず実行）
    predict(a_meas, omega_meas);

    // 2. エンコーダ更新（必ず実行）
    updateEncoder(v_R, v_L, omega_meas);

    if (fix == nullptr) return;

    // 3. GNSS位置更新（新しいfixが届いたときのみ）
    double pN, pE;
    latLonToLocal(fix->lat, fix->lon, pN, pE);
    updateGnssPosition(pN, pE);
    gnss_updated = true;

    // 4. COG更新（同一fix かつ SOG ≥ v_threshold のときのみ）
    if (cogUpdateEnabled(fix->sog)) {
        updateCog(fix->cog_deg * DEG2RAD);
        cog_updated = true;
    }
}

// =============================================================================
// 予測ステップ（1.2〜1.4節、ekf_predict.m）
// =============================================================================
void KalmanFilter::predict(double a_meas, double omega_meas) {
    const double dt = cfg.dt;
    const double v = x[V], th = x[THETA];
    const double c = cos(th), s = sin(th);

    // ヤコビアン F は直前の推定値 v̂, θ̂ で評価する（1.3節）
    double F[N][N] = {{0}};
    for (int i = 0; i < N; i++) F[i][i] = 1;
    F[PN][V]     = c * dt;
    F[PN][THETA] = -v * s * dt;
    F[PE][V]     = s * dt;
    F[PE][THETA] = v * c * dt;
    F[V][BA]     = dt;
    F[THETA][BG] = dt;

    // 状態の予測
    x[PN]    += v * c * dt;
    x[PE]    += v * s * dt;
    x[V]     -= (a_meas - x[BA]) * dt;
    x[THETA]  = wrapToPi(x[THETA] - (omega_meas - x[BG]) * dt);
    // b_a, b_g は不変

    // P = F・P・F^T + Q
    double FP[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            double sum = 0;
            for (int k = 0; k < N; k++) sum += F[i][k] * P[k][j];
            FP[i][j] = sum;
        }
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            double sum = 0;
            for (int k = 0; k < N; k++) sum += FP[i][k] * F[j][k];
            P[i][j] = sum + Q[i][j];
        }
}

// =============================================================================
// 観測更新
// =============================================================================

// エンコーダ（1.5節）：h_enc(x) = [v + (L/2)(ω_meas − b_g), v - (L/2)(ω_meas − b_g)]
void KalmanFilter::updateEncoder(double v_R, double v_L, double omega_meas) {
    const double w_corr = omega_meas - x[BG];
    const double y[2] = {
        v_R - (x[V] + cfg.L / 2 * w_corr),
        v_L - (x[V] - cfg.L / 2 * w_corr),
    };
    double S[2][2];
    applyInnovation<2>(H_enc, y, R_enc, S);
    y_enc[0] = y[0];  y_enc[1] = y[1];
    S_enc[0] = S[0][0];  S_enc[1] = S[1][1];
}

// GNSS位置（1.6節）：h_gnss(x) = [p_N, p_E]
void KalmanFilter::updateGnssPosition(double p_N_gnss, double p_E_gnss) {
    const double y[2] = { p_N_gnss - x[PN], p_E_gnss - x[PE] };
    double S[2][2];
    applyInnovation<2>(H_gnss, y, R_gnss, S);
    y_gnss[0] = y[0];  y_gnss[1] = y[1];
    S_gnss[0] = S[0][0];  S_gnss[1] = S[1][1];
}

// GNSS COG（1.7節）：h_cog(x) = θ。イノベーションは必ず (-π, π] に折り返す
void KalmanFilter::updateCog(double cog_rad) {
    const double y[1] = { wrapToPi(cog_rad - x[THETA]) };
    double S[1][1];
    applyInnovation<1>(H_cog, y, R_cog, S);
    x[THETA] = wrapToPi(x[THETA]);
    y_cog = y[0];
    S_cog = S[0][0];
}

template <int M>
void KalmanFilter::applyInnovation(const double H[M][N], const double y[M], const double R[M][M],
                                   double S[M][M]) {
    // PHt = P・H^T  (N×M)
    double PHt[N][M];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < M; j++) {
            double sum = 0;
            for (int k = 0; k < N; k++) sum += P[i][k] * H[j][k];
            PHt[i][j] = sum;
        }

    // S = H・P・H^T + R  (M×M)
    for (int i = 0; i < M; i++)
        for (int j = 0; j < M; j++) {
            double sum = 0;
            for (int k = 0; k < N; k++) sum += H[i][k] * PHt[k][j];
            S[i][j] = sum + R[i][j];
        }

    // S^-1（M = 1, 2 のみ対応）
    double Sinv[M][M];
    invert(S, Sinv);

    // K = P・H^T・S^-1  (N×M)
    double K[N][M];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < M; j++) {
            double sum = 0;
            for (int k = 0; k < M; k++) sum += PHt[i][k] * Sinv[k][j];
            K[i][j] = sum;
        }

    // x = x + K・y
    for (int i = 0; i < N; i++) {
        double sum = 0;
        for (int k = 0; k < M; k++) sum += K[i][k] * y[k];
        x[i] += sum;
    }

    // P = (I − K・H)・P = P − K・(H・P)。H・P = (P・H^T)^T（P は対称）
    double newP[N][N];
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++) {
            double sum = 0;
            for (int k = 0; k < M; k++) sum += K[i][k] * PHt[j][k];
            newP[i][j] = P[i][j] - sum;
        }
    // 数値誤差による非対称化を防ぐ
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            P[i][j] = (newP[i][j] + newP[j][i]) / 2;
}

// =============================================================================
// ユーティリティ
// =============================================================================
void KalmanFilter::latLonToLocal(double lat, double lon, double& p_N, double& p_E) const {
    p_N = (lat - lat0) * EARTH_M_PER_DEG;
    p_E = (lon - lon0) * EARTH_M_PER_DEG * cos_lat0;
}

double KalmanFilter::wrapToPi(double a) {
    // mod(a + π, 2π) − π。結果は [-π, π)（-π と π は同一方位なので実用上問題なし）
    a = fmod(a + M_PI, 2 * M_PI);
    if (a < 0) a += 2 * M_PI;
    return a - M_PI;
}
