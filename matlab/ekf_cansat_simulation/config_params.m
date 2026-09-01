function params = config_params()
% CONFIG_PARAMS  CanSat 2輪走行EKFシミュレーションの全パラメータを一元管理する。
%
%   このファイルだけを編集すればシミュレーション条件を変更できる。
%   数式・行列構造そのものは EKF_MATLAB_simulation_requirements.md で確定済みのため、
%   他ファイルにハードコードしないこと。

%% ---- 周期（要件書3章: 確定値） -------------------------------------
params.dt = 0.05;              % [s] EKF予測・エンコーダ更新の周期 (20 Hz)
params.gnss_period_steps = 20; % [steps] 何ステップに1回GNSS fixが届くか (dt*20 = 1.0s -> 約1Hz)

%% ---- シミュレーションフェーズ長 -------------------------------------
params.T_phase0 = 10;          % [s] Phase0（静止、EKF初期化用）の長さ
params.T_calib_straight = 10;  % [s] Phase0直後、初期方位角較正のためだけに直進する時間

%% ---- 直進・方向転換の速度 --------------------------------------------
% 加減速は実機挙動を想定し、ほぼ0秒（1ステップ=dt）で巡航速度に到達/停止する
% bang-bangモデル（simulate_segment.m内で v_cruise/dt のスパイクとして実装）。
params.v_cruise = 0.3;                  % [m/s] 巡航速度
params.turn_omega_target = deg2rad(45); % [rad/s] 方向転換時の目標角速度（左右輪駆動想定）

%% ---- 機体寸法（後日実測値に差し替え） --------------------------------
params.r = 0.045;    % [m] タイヤ半径
params.L = 0.20;    % [m] トレッド幅

%% ---- センサノイズ・バイアス（後日実測値に差し替え） -------------------
params.sigma_theta_R = deg2rad(0.5);   % [rad] 右輪AS5600 角度読み取りノイズ標準偏差
params.sigma_theta_L = deg2rad(0.5);   % [rad] 左輪AS5600 角度読み取りノイズ標準偏差

params.sigma_a    = 0.26;              % [m/s^2] 加速度計ノイズ標準偏差
params.sigma_rw_a = 0.001;             % [m/s^2/sqrt(s)] 加速度計バイアスのランダムウォーク係数

params.sigma_g    = deg2rad(0.05);      % [rad/s] ジャイロノイズ標準偏差
params.sigma_rw_g = deg2rad(0.01);     % [rad/s/sqrt(s)] ジャイロバイアスのランダムウォーク係数

params.sigma_N = 2.12;   % [m] GNSS 北方向位置ノイズ標準偏差
params.sigma_E = 2.12;   % [m] GNSS 東方向位置ノイズ標準偏差

%% ---- 真のバイアス初期値（シナリオ生成用、後日実測値に差し替え） -------
% EKFのb_a, b_gが「動くバイアス」を追従できるかを検証するため、
% 真値側のバイアスも sigma_rw_a / sigma_rw_g と同じスケールでランダムウォークさせる。
params.b_a_true_init = 0.02;           % [m/s^2]
params.b_g_true_init = deg2rad(0.3);   % [rad/s]

%% ---- 真の初期方位角（シナリオ生成用） ---------------------------------
% 電源投入時に機体が実際にどちらを向いているかは、静止しているPhase0中の
% GNSSだけではわからない（COGは移動していないと出ないため）。EKF側はこの値を
% 直接は知らず、Phase0直後のキャリブレーション走行（T_calib_straight、
% simulate_goal_directed.m参照）でGNSS変位から推定した値を初期推定として使う。
params.theta_true_init_deg = 130; % [deg] 真の初期方位角（真北基準・時計回り正）

%% ---- GNSS 基準点（緯度経度原点、後日実測値に差し替え） -----------------
params.lat0 = 35.0;   % [deg]
params.lon0 = 139.0;  % [deg]
params.EARTH_M_PER_DEG = 111320; % [m/deg] 緯度1度あたりの距離の近似値（要件書1.6節、確定値）

%% ---- EKF初期共分散 P0（後日実測値に差し替え） -------------------------
% theta（4番目）はキャリブレーション走行（T_calib_straight）でのGNSS変位から
% 推定した方位角の誤差分散に simulate_goal_directed.m 内で置き換えられる。
% ここの値はキャリブレーションが使えない（GNSS fixが2点未満など）場合の
% フォールバック用。
params.P0_diag_theta_fallback = deg2rad(90)^2;
params.P0_diag_theta_floor    = deg2rad(5)^2;  % キャリブレーション結果を過信しすぎないための下限
params.P0_diag = [ (2*params.sigma_N)^2, (2*params.sigma_E)^2, 0.2^2, params.P0_diag_theta_fallback, 0.1^2, deg2rad(1)^2 ];

%% ---- 乱数シード（確定値: 再現性確保のため固定） ------------------------
params.rng_seed = 43;

end
