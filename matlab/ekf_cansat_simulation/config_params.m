function params = config_params()
% CONFIG_PARAMS  CanSat 2輪誘導 EKF + 方位PID シミュレーションの全パラメータを一元管理する。
%
%   要件書「EKF設計及びシミュレーション方法.md」3章のパラメータをすべてここに集約する。
%   数式・行列構造（要件書1章）は確定済みのため、他ファイルに数値をハードコードしないこと。
%
%   【旧】と付いた値は、旧シミュレーション（レグ方式の目的地誘導版）から引き継いだもの。

%% ==== 3.1 周期 ==========================================================
params.dt          = 0.1;   % [s] EKF予測・エンコーダ更新・制御の周期（既定 10 Hz。0.05 も試せる）
params.gnss_period = 1.0;   % [s] GNSS fix の到着周期（位置・COG・SOG が同一 fix で届く）

%% ==== 3.2 センサ・EKF（後日実測値に差し替える仮値） ======================
% --- エンコーダ（AS5600、左右別個体） ---
params.sigma_theta_R      = deg2rad(0.5); % [rad] 右輪 角度読み取りノイズ【旧】
params.sigma_theta_L      = deg2rad(0.5); % [rad] 左輪 角度読み取りノイズ【旧】
params.enc_counts_per_rev = 5880;         % [counts/rev] 角度分解能 2π/5880（要件書1.5節）

% --- IMU（MPU6050） ---
params.sigma_a    = 0.26;           % [m/s^2] 加速度計白色雑音（400 μg/√Hz × √44 Hz）【旧】
params.sigma_rw_a = 0.001;          % [m/s^2/√s] 加速度計バイアス ランダムウォーク係数【旧】
params.sigma_g    = deg2rad(0.05);  % [rad/s] ジャイロ白色雑音（0.005 °/s/√Hz × √100 Hz）【旧】
params.sigma_rw_g = deg2rad(0.01);  % [rad/s/√s] ジャイロバイアス ランダムウォーク係数【旧】

% --- GNSS（NEO-6M） ---
params.sigma_N   = 2.12;          % [m] 北方向 位置ノイズ（CEP 2.5 m / 1.1774）【旧】
params.sigma_E   = 2.12;          % [m] 東方向 位置ノイズ【旧】
params.sigma_cog = deg2rad(0.5);  % [rad] COG ノイズ（NEO-6 Heading accuracy ±0.5°）
params.sigma_sog = 0.1;           % [m/s] SOG ノイズ（NEO-6 速度精度 0.1 m/s）
params.v_threshold = 0.2;         % [m/s] COG 更新を有効とみなす最小 SOG（仮値。走行試験で調整）
params.cog_v_eps   = 0.01;        % [m/s] 疑似COG生成時の σ_sog/max(v,ε) のゼロ割り防止

% --- 疑似観測に乗せる一定バイアス（2.6節） ---
params.b_a_const = 0.02;          % [m/s^2]【旧 b_a_true_init】
params.b_g_const = deg2rad(0.3);  % [rad/s]【旧 b_g_true_init】

% --- 初期共分散 P0 の設定値（2.2節） ---
params.alpha    = 3;      % [-] GNSS 平均の安全係数（2〜4）
params.sigma_v0 = 0.01;   % [m/s] 静止中の速度の不確かさ
% σ_ba0, σ_bg0 は「静止区間の IMU 平均の標準誤差 × bias_se_margin」で自動算出する。
% 固定値を使いたい場合は [] を数値に置き換える。
params.sigma_ba0 = [];            % [m/s^2]
params.sigma_bg0 = [];            % [rad/s]
params.bias_se_margin = 3;        % [-] 標準誤差に掛ける余裕係数
params.P0_theta_max = pi^2;       % [rad^2] D が極端に小さいとき P0(θ) が発散しないための上限

%% ==== 3.3 機体・モーター ================================================
params.r          = 0.045;  % [m] タイヤ半径【旧】
params.L          = 0.20;   % [m] トレッド幅【旧】
params.rpm_noload = 71;     % [rpm] GA12-N20 無負荷回転数
params.eta_load   = 0.8;    % [-] 負荷時の回転数低下率（仮値）
params.v_wheel_max = params.eta_load * (2*pi*params.rpm_noload/60) * params.r; % [m/s] ≈ 0.268

%% ==== 3.4 シナリオ ======================================================
params.theta_model0 = deg2rad(130); % [rad] 運動モデルの初期方位角（機体は知らない）【旧 theta_true_init_deg】
params.N_avg = 10;                  % [回] 測位の平均回数
params.v_cal = 0.25;                % [m/s] キャリブレーション直進速度（≤ v_wheel_max）
params.t_cal = 10;                  % [s] キャリブレーション直進時間【旧 T_calib_straight】
params.t_settle = 0.5;              % [s] Phase 2 冒頭のバイアス平均から除外する時間（停止時の加速度スパイク除去）

% ゴール：出発地点原点のローカル座標。距離・方位で与える【旧 50 m @ 55°】
params.goal_distance = 50;          % [m]
params.goal_bearing  = deg2rad(55); % [rad] 真北基準・時計回り
params.N_goal = params.goal_distance * cos(params.goal_bearing);
params.E_goal = params.goal_distance * sin(params.goal_bearing);

params.t_max  = 600;  % [s] 誘導（Phase 3）のタイムアウト
params.t_post = 5;    % [s] 停止後の記録時間（0 で即終了）
params.rng_seed = 43; % 乱数シード【旧】

% GNSS 緯度経度 ⇔ ローカル座標変換（1.6節）【旧】
params.lat0 = 35.0;               % [deg] 疑似観測を緯度経度で作るための基準点
params.lon0 = 139.0;              % [deg]
params.EARTH_M_PER_DEG = 111320;  % [m/deg]

%% ==== 3.5 制御 ==========================================================
params.Kp    = 1.0;   % [1/s] 方位応答の時定数 ≈ 1/Kp
params.Ki    = 0;     % [1/s^2] 初期値は PD 制御
params.Kd    = 0.2;   % [s]
params.I_max = 0.5;   % [rad·s] 積分クランプ
params.omega_cmd_max = deg2rad(45); % [rad/s] 旋回角速度指令上限【旧 turn_omega_target】(≤ 2·v_wheel_max/L)
params.v_cruise = 0.2;              % [m/s] 巡航速度（旧 0.3 は v_wheel_max を超えるため変更）
params.d_stop = 3;    % [m] 急停止判定距離
params.N_stop = 3;    % [回] 連続成立ループ数

%% ==== 整合性チェック =====================================================
assert(params.omega_cmd_max <= 2*params.v_wheel_max/params.L, ...
    'omega_cmd_max は 2·v_wheel_max/L (= %.3f rad/s) 以下にすること', 2*params.v_wheel_max/params.L);
assert(params.v_cal <= params.v_wheel_max, 'v_cal が v_wheel_max を超えている');
assert(params.v_cruise <= params.v_wheel_max, 'v_cruise が v_wheel_max を超えている');
assert(abs(params.gnss_period/params.dt - round(params.gnss_period/params.dt)) < 1e-9, ...
    'gnss_period は dt の整数倍にすること');

end
