function [T, PHASE, gt_state, X_est, P_hist, leg_end_idx, turn_pos_est, turn_theta_est] = simulate_goal_directed(params, use_encoder)
% SIMULATE_GOAL_DIRECTED  目的地誘導シナリオ本体。
%   Phase0（静止）でEKFを初期化した後、以下を目的地に着くまで繰り返す。
%     1) その時点のEKF推定位置（実機が使える唯一の情報。真値は使わない）
%        から見た目的地への方位角を計算し、そちらを向くよう方向転換する。
%     2) 1レグ分（leg_duration_s秒）だけ直進する（bang-bang加減速、実機挙動想定）。
%
%   use_encoder = false にすると、EKFのエンコーダ更新ステップを丸ごとスキップし、
%   予測とGNSS更新（約1Hz）だけで推定する（エンコーダなし比較用）。
%   run_goal_directed_simulation.m / run_goal_directed_simulation_no_encoder.m
%   から呼ばれる（呼び出し側は rng(params.rng_seed) を先に実行しておくこと）。
%
%   出力:
%     T, PHASE                 : 1xN 時刻・フェーズラベル("phase0"/"turn"/"straight")
%     gt_state                 : 6xN 真値 [p_N;p_E;v;theta;b_a_true;b_g_true]
%     X_est, P_hist             : EKF推定値・共分散履歴
%     leg_end_idx               : 1xM 各レグ（方向転換+直進）が完了した時点のグローバル
%                                  インデックス（M=完了レグ数）。先頭Nレグだけの
%                                  軌道を描くのに使う。
%     turn_pos_est, turn_theta_est : 2xM, 1xM。各レグの方向転換「直前」（=その時点で
%                                  誘導ロジックが使った）EKF推定位置・推定方位角。
%                                  軌道図に矢印で重ねて表示するのに使う。

dt = params.dt;

%% --- EKF行列（一定、要件書1.3〜1.6節） ---
Q = diag([0, 0, params.sigma_a^2*dt^2, 0, ...
          params.sigma_rw_a^2*dt, params.sigma_rw_g^2*dt]);
H_gnss = [1 0 0 0 0 0;
          0 1 0 0 0 0];
R_gnss = diag([params.sigma_N^2, params.sigma_E^2]);
H_enc = [0 0 1 0 0 -params.L/2;
         0 0 1 0 0  params.L/2];
R_enc = diag([params.r^2 * 2*params.sigma_theta_R^2/dt^2, ...
              params.r^2 * 2*params.sigma_theta_L^2/dt^2]);

%% --- 履歴バッファ（十分大きめに確保し、最後に実長へトリムする） ---
MAXN = 30000;
T = zeros(1, MAXN);
PHASE = strings(1, MAXN);
GT_pN = zeros(1, MAXN); GT_pE = zeros(1, MAXN); GT_v = zeros(1, MAXN); GT_th = zeros(1, MAXN);
GT_ba = zeros(1, MAXN); GT_bg = zeros(1, MAXN);
GNSS_FIX = false(1, MAXN);
A_MEAS = zeros(1, MAXN); OMEGA_MEAS = zeros(1, MAXN);
COUNTS_R = zeros(1, MAXN); COUNTS_L = zeros(1, MAXN);
Z_GNSS_N = zeros(1, MAXN); Z_GNSS_E = zeros(1, MAXN);
X_est = zeros(6, MAXN);
P_hist = zeros(6, 6, MAXN);
leg_end_idx = zeros(1, params.max_legs);

%% --- Phase0: 静止（EKF初期化のための区間） ---
% 真の初期方位角は params.theta_true_init_deg（EKFの初期推定=0とは意図的に
% 異なる値。静止中のGNSSだけでは実機の向きはわからないため）。
n0 = round(params.T_phase0 / dt);
a0 = zeros(1, n0);
w0 = zeros(1, n0);
theta_true0 = deg2rad(params.theta_true_init_deg);
seg = simulate_segment(a0, w0, [0;0;0;theta_true0], [params.b_a_true_init; params.b_g_true_init], [0;0], 0, params);

rng_idx = 1:n0;
T(rng_idx) = (0:n0-1) * dt;
PHASE(rng_idx) = "phase0";
GT_pN(rng_idx) = seg.p_N; GT_pE(rng_idx) = seg.p_E; GT_v(rng_idx) = seg.v; GT_th(rng_idx) = seg.theta;
GT_ba(rng_idx) = seg.b_a_true; GT_bg(rng_idx) = seg.b_g_true;
GNSS_FIX(rng_idx) = seg.gnss_fix;
A_MEAS(rng_idx) = seg.a_meas; OMEGA_MEAS(rng_idx) = seg.omega_meas;
COUNTS_R(rng_idx) = seg.counts_R; COUNTS_L(rng_idx) = seg.counts_L;
Z_GNSS_N(rng_idx) = seg.z_gnss_N; Z_GNSS_E(rng_idx) = seg.z_gnss_E;

idx = n0;

state_cur = [GT_pN(idx); GT_pE(idx); GT_v(idx); GT_th(idx)];
bias_cur  = [GT_ba(idx); GT_bg(idx)];
wheel_cur = seg.end_wheel;

%% --- キャリブレーション走行: 直進してGNSS変位から初期方位角を推定 ---
% 旋回は一切かけず、Phase0終了時点の（未知の）真の向きのまま直進する。
% 得られたGNSS fixの位置を時間の1次回帰にかけ、その傾き(dN/dt, dE/dt)から
% 方位角を求める。回帰は複数のGNSS点を使うぶん、2点の差分より雑音に強い。
n_calib = round(params.T_calib_straight / dt);
a_calib = zeros(1, n_calib);
if n_calib >= 2
    a_calib(1)   = params.v_cruise / dt;
    a_calib(end) = -params.v_cruise / dt;
end
w_calib = zeros(1, n_calib);

seg = simulate_segment(a_calib, w_calib, state_cur, bias_cur, wheel_cur, idx, params);
rng_idx = idx+1 : idx+n_calib;
T(rng_idx) = (idx : idx+n_calib-1) * dt;
PHASE(rng_idx) = "calib";
GT_pN(rng_idx) = seg.p_N; GT_pE(rng_idx) = seg.p_E; GT_v(rng_idx) = seg.v; GT_th(rng_idx) = seg.theta;
GT_ba(rng_idx) = seg.b_a_true; GT_bg(rng_idx) = seg.b_g_true;
GNSS_FIX(rng_idx) = seg.gnss_fix;
A_MEAS(rng_idx) = seg.a_meas; OMEGA_MEAS(rng_idx) = seg.omega_meas;
COUNTS_R(rng_idx) = seg.counts_R; COUNTS_L(rng_idx) = seg.counts_L;
Z_GNSS_N(rng_idx) = seg.z_gnss_N; Z_GNSS_E(rng_idx) = seg.z_gnss_E;

idx = idx + n_calib;

calib_fix_idx = rng_idx(seg.gnss_fix);
if numel(calib_fix_idx) >= 2
    t_fit = T(calib_fix_idx) - T(calib_fix_idx(1));
    pN_fit = polyfit(t_fit, Z_GNSS_N(calib_fix_idx), 1);
    pE_fit = polyfit(t_fit, Z_GNSS_E(calib_fix_idx), 1);
    dN = pN_fit(1); dE = pE_fit(1); % 推定並進速度ベクトル [m/s]

    theta_cal = atan2(dE, dN);

    Sxx = sum((t_fit - mean(t_fit)).^2);
    sigma_dN = params.sigma_N / sqrt(Sxx);
    sigma_dE = params.sigma_E / sqrt(Sxx);
    % atan2(dE,dN)の誤差伝播（デルタ法）
    sigma_theta_cal = sqrt(dN^2*sigma_dE^2 + dE^2*sigma_dN^2) / (dN^2 + dE^2);
    sigma_theta_cal = max(sigma_theta_cal, sqrt(params.P0_diag_theta_floor));

    fprintf('初期方位角キャリブレーション: 推定 %.1f deg (真値 %.1f deg, 誤差 %.1f deg), 1sigma=%.1f deg (GNSS fix %d点)\n', ...
        rad2deg(theta_cal), params.theta_true_init_deg, ...
        rad2deg(wrap_to_pi_local(theta_cal - deg2rad(params.theta_true_init_deg))), ...
        rad2deg(sigma_theta_cal), numel(calib_fix_idx));
else
    theta_cal = 0;
    sigma_theta_cal = sqrt(params.P0_diag_theta_fallback);
    warning('simulate_goal_directed:calibFailed', ...
        'Calibration straight run has fewer than 2 GNSS fixes; falling back to theta0=0 with a wide P0.');
end

%% --- EKF初期化（Phase0のGNSS平均＋キャリブレーション走行の方位角推定） ---
idx0 = find(PHASE(1:n0) == "phase0" & GNSS_FIX(1:n0));
P0_diag = params.P0_diag;
P0_diag(4) = sigma_theta_cal^2;
x0 = [mean(Z_GNSS_N(idx0)); mean(Z_GNSS_E(idx0)); 0; theta_cal; 0; 0];
P0 = diag(P0_diag);
X_est(:,1) = x0;
P_hist(:,:,1) = P0;

[X_est, P_hist] = run_ekf_over_range(X_est, P_hist, 1, idx, ...
    A_MEAS, OMEGA_MEAS, COUNTS_R, COUNTS_L, GNSS_FIX, Z_GNSS_N, Z_GNSS_E, ...
    dt, Q, H_enc, R_enc, H_gnss, R_gnss, params, use_encoder);

state_cur = [GT_pN(idx); GT_pE(idx); GT_v(idx); GT_th(idx)];
bias_cur  = [GT_ba(idx); GT_bg(idx)];
wheel_cur = seg.end_wheel;

%% --- 目的地誘導ループ ---
wrap_to_pi = @(a) mod(a + pi, 2*pi) - pi;
leg = 0;
turn_pos_est = zeros(2, params.max_legs);
turn_theta_est = zeros(1, params.max_legs);

while leg < params.max_legs
    x_hat = X_est(:, idx);
    dist_to_goal = hypot(params.goal_N - x_hat(1), params.goal_E - x_hat(2));
    if dist_to_goal <= params.arrival_tolerance_m
        break;
    end
    leg = leg + 1;

    % 方向転換「直前」に誘導ロジックが実際に使ったEKF推定位置・方位角を記録
    % （軌道図への矢印表示用）。
    turn_pos_est(:, leg) = x_hat(1:2);
    turn_theta_est(leg) = x_hat(4);

    %% 1) 方向転換: EKF推定位置から見た目的地方位へ補正
    target_bearing = atan2(params.goal_E - x_hat(2), params.goal_N - x_hat(1));
    turn_needed = wrap_to_pi(target_bearing - x_hat(4));
    n_turn = max(1, round(abs(turn_needed) / params.turn_omega_target / dt));

    a_turn = zeros(1, n_turn);
    w_turn = sign(turn_needed) * params.turn_omega_target * ones(1, n_turn);

    if idx + n_turn > MAXN - 1000
        warning('simulate_goal_directed:bufferNearFull', ...
            'History buffer nearing MAXN; stopping early before leg %d (turn).', leg);
        leg = leg - 1;
        break;
    end

    seg = simulate_segment(a_turn, w_turn, state_cur, bias_cur, wheel_cur, idx, params);
    rng_idx = idx+1 : idx+n_turn;
    T(rng_idx) = (idx : idx+n_turn-1) * dt;
    PHASE(rng_idx) = "turn";
    GT_pN(rng_idx) = seg.p_N; GT_pE(rng_idx) = seg.p_E; GT_v(rng_idx) = seg.v; GT_th(rng_idx) = seg.theta;
    GT_ba(rng_idx) = seg.b_a_true; GT_bg(rng_idx) = seg.b_g_true;
    GNSS_FIX(rng_idx) = seg.gnss_fix;
    A_MEAS(rng_idx) = seg.a_meas; OMEGA_MEAS(rng_idx) = seg.omega_meas;
    COUNTS_R(rng_idx) = seg.counts_R; COUNTS_L(rng_idx) = seg.counts_L;
    Z_GNSS_N(rng_idx) = seg.z_gnss_N; Z_GNSS_E(rng_idx) = seg.z_gnss_E;

    [X_est, P_hist] = run_ekf_over_range(X_est, P_hist, idx, idx+n_turn, ...
        A_MEAS, OMEGA_MEAS, COUNTS_R, COUNTS_L, GNSS_FIX, Z_GNSS_N, Z_GNSS_E, ...
        dt, Q, H_enc, R_enc, H_gnss, R_gnss, params, use_encoder);

    idx = idx + n_turn;
    state_cur = [GT_pN(idx); GT_pE(idx); GT_v(idx); GT_th(idx)];
    bias_cur  = [GT_ba(idx); GT_bg(idx)];
    wheel_cur = seg.end_wheel;

    %% 2) 直進: 1レグ分（bang-bang加減速、実機挙動想定。時間ベースでleg_duration_s秒）
    n_leg = max(3, round(params.leg_duration_s / dt));
    a_leg = zeros(1, n_leg);
    a_leg(1)   = params.v_cruise / dt;   % ほぼ0秒で加速
    a_leg(end) = -params.v_cruise / dt;  % ほぼ0秒で減速・停止
    w_leg = zeros(1, n_leg);

    if idx + n_leg > MAXN - 1000
        warning('simulate_goal_directed:bufferNearFull', ...
            'History buffer nearing MAXN; stopping early before leg %d (straight).', leg);
        break;
    end

    seg = simulate_segment(a_leg, w_leg, state_cur, bias_cur, wheel_cur, idx, params);
    rng_idx = idx+1 : idx+n_leg;
    T(rng_idx) = (idx : idx+n_leg-1) * dt;
    PHASE(rng_idx) = "straight";
    GT_pN(rng_idx) = seg.p_N; GT_pE(rng_idx) = seg.p_E; GT_v(rng_idx) = seg.v; GT_th(rng_idx) = seg.theta;
    GT_ba(rng_idx) = seg.b_a_true; GT_bg(rng_idx) = seg.b_g_true;
    GNSS_FIX(rng_idx) = seg.gnss_fix;
    A_MEAS(rng_idx) = seg.a_meas; OMEGA_MEAS(rng_idx) = seg.omega_meas;
    COUNTS_R(rng_idx) = seg.counts_R; COUNTS_L(rng_idx) = seg.counts_L;
    Z_GNSS_N(rng_idx) = seg.z_gnss_N; Z_GNSS_E(rng_idx) = seg.z_gnss_E;

    [X_est, P_hist] = run_ekf_over_range(X_est, P_hist, idx, idx+n_leg, ...
        A_MEAS, OMEGA_MEAS, COUNTS_R, COUNTS_L, GNSS_FIX, Z_GNSS_N, Z_GNSS_E, ...
        dt, Q, H_enc, R_enc, H_gnss, R_gnss, params, use_encoder);

    idx = idx + n_leg;
    state_cur = [GT_pN(idx); GT_pE(idx); GT_v(idx); GT_th(idx)];
    bias_cur  = [GT_ba(idx); GT_bg(idx)];
    wheel_cur = seg.end_wheel;

    leg_end_idx(leg) = idx;

    if idx > MAXN - 1000
        warning('simulate_goal_directed:bufferNearFull', ...
            'History buffer nearing MAXN; stopping early at leg %d.', leg);
        break;
    end
end

fprintf('目的地誘導ループ終了(%s): %d レグ, 最終ステップ %d (t=%.1fs), 目的地までの距離(EKF推定) %.2fm\n', ...
    ternary_str(use_encoder, 'encoder+GNSS', 'GNSS only'), leg, idx, T(idx), ...
    hypot(params.goal_N - X_est(1,idx), params.goal_E - X_est(2,idx)));

%% --- トリム ---
N = idx;
T = T(1:N); PHASE = PHASE(1:N);
GT_pN = GT_pN(1:N); GT_pE = GT_pE(1:N); GT_v = GT_v(1:N); GT_th = GT_th(1:N);
GT_ba = GT_ba(1:N); GT_bg = GT_bg(1:N);
X_est = X_est(:,1:N);
P_hist = P_hist(:,:,1:N);
leg_end_idx = leg_end_idx(1:leg);
turn_pos_est = turn_pos_est(:,1:leg);
turn_theta_est = turn_theta_est(1:leg);

gt_state = [GT_pN; GT_pE; GT_v; GT_th; GT_ba; GT_bg];

end


function s = ternary_str(cond, a, b)
if cond
    s = a;
else
    s = b;
end
end


function a = wrap_to_pi_local(a)
a = mod(a + pi, 2*pi) - pi;
end
