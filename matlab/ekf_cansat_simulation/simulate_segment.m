function seg = simulate_segment(a_seg, w_seg, state0, bias0, wheel0, start_global_step, params)
% SIMULATE_SEGMENT  与えられた制御入力列(a_seg, w_seg)に沿って、真の軌道と
%   疑似観測値を1ステップずつ生成する（generate_ground_truth.m /
%   generate_sensor_measurements.m と同じ物理・センサモデルを、任意の初期状態
%   から続けて呼び出せる形に切り出したもの）。
%
%   目的地誘導シナリオのように、次のセグメント（直進 or 方向転換）の長さや
%   方位が「その時点のEKF推定値」に依存して実行時に決まる場合、全区間を
%   事前に一括生成できないため、区間ごとに呼び出して真値・観測値を継ぎ足す。
%
%   入力:
%     a_seg, w_seg       : 1xn 真の加速度・角速度（このセグメントの制御入力）
%     state0              : [p_N;p_E;v;theta] このセグメント開始直前の真値状態
%     bias0               : [b_a;b_g] このセグメント開始直前の真のIMUバイアス
%     wheel0              : [wheel_angle_R;wheel_angle_L] 開始直前の連続ホイール角[rad]
%     start_global_step   : state0が対応するグローバルステップ番号（GNSS fix
%                            タイミング mod(global_step-1, gnss_period_steps)==0
%                            の判定に使う。シミュレーション全体で共通の通し番号）
%     params              : config_params()相当の構造体
%
%   出力 seg (struct, 各フィールドは 1xn。GNSS未到来点はNaN):
%     p_N,p_E,v,theta, b_a_true,b_g_true, a_meas,omega_meas,
%     counts_R,counts_L, lat,lon,z_gnss_N,z_gnss_E, gnss_fix
%     end_state, end_bias, end_wheel, end_counts : 次セグメントへの引き継ぎ用

dt = params.dt;
n = numel(a_seg);
COUNTS_PER_REV = 4096;

p_N = zeros(1,n); p_E = zeros(1,n); v = zeros(1,n); theta = zeros(1,n);
b_a_true = zeros(1,n); b_g_true = zeros(1,n);
a_meas = zeros(1,n); omega_meas = zeros(1,n);
counts_R = zeros(1,n); counts_L = zeros(1,n);
lat = nan(1,n); lon = nan(1,n); z_gnss_N = nan(1,n); z_gnss_E = nan(1,n);
gnss_fix = false(1,n);

p_N_prev = state0(1); p_E_prev = state0(2); v_prev = state0(3); theta_prev = state0(4);
b_a_prev = bias0(1); b_g_prev = bias0(2);
wheel_R = wheel0(1); wheel_L = wheel0(2);

for i = 1:n
    a_k = a_seg(i);
    w_k = w_seg(i);

    % --- 真の運動学（要件書1.2節と同じ式） ---
    p_N(i) = p_N_prev + v_prev * cos(theta_prev) * dt;
    p_E(i) = p_E_prev + v_prev * sin(theta_prev) * dt;
    v(i)   = v_prev + a_k * dt;
    theta(i) = theta_prev + w_k * dt;

    % --- 真のIMUバイアス（ランダムウォーク継続） ---
    b_a_true(i) = b_a_prev + randn() * params.sigma_rw_a * sqrt(dt);
    b_g_true(i) = b_g_prev + randn() * params.sigma_rw_g * sqrt(dt);

    % --- IMU疑似観測値 ---
    a_meas(i) = a_k + b_a_prev + params.sigma_a * randn();
    omega_meas(i) = w_k + b_g_prev + params.sigma_g * randn();

    % --- エンコーダ（左右輪AS5600） ---
    v_R_true = v_prev + (params.L/2) * w_k;
    v_L_true = v_prev - (params.L/2) * w_k;
    wheel_R = wheel_R + v_R_true / params.r * dt;
    wheel_L = wheel_L + v_L_true / params.r * dt;
    counts_R(i) = mod(round((wheel_R + params.sigma_theta_R*randn()) / (2*pi) * COUNTS_PER_REV), COUNTS_PER_REV);
    counts_L(i) = mod(round((wheel_L + params.sigma_theta_L*randn()) / (2*pi) * COUNTS_PER_REV), COUNTS_PER_REV);

    % --- GNSS（グローバルステップ基準で約1Hz） ---
    global_step = start_global_step + i;
    if mod(global_step - 1, params.gnss_period_steps) == 0
        gnss_fix(i) = true;
        [true_lat, true_lon] = local_to_latlon(p_N(i), p_E(i), params.lat0, params.lon0, params.EARTH_M_PER_DEG);
        lat(i) = true_lat + (params.sigma_N * randn()) / params.EARTH_M_PER_DEG;
        lon(i) = true_lon + (params.sigma_E * randn()) / (params.EARTH_M_PER_DEG * cos(deg2rad(params.lat0)));
        [z_gnss_N(i), z_gnss_E(i)] = latlon_to_local(lat(i), lon(i), params.lat0, params.lon0, params.EARTH_M_PER_DEG);
    end

    p_N_prev = p_N(i); p_E_prev = p_E(i); v_prev = v(i); theta_prev = theta(i);
    b_a_prev = b_a_true(i); b_g_prev = b_g_true(i);
end

seg.n = n;
seg.p_N = p_N; seg.p_E = p_E; seg.v = v; seg.theta = theta;
seg.b_a_true = b_a_true; seg.b_g_true = b_g_true;
seg.a_meas = a_meas; seg.omega_meas = omega_meas;
seg.counts_R = counts_R; seg.counts_L = counts_L;
seg.lat = lat; seg.lon = lon; seg.z_gnss_N = z_gnss_N; seg.z_gnss_E = z_gnss_E;
seg.gnss_fix = gnss_fix;

seg.end_state  = [p_N(n); p_E(n); v(n); theta(n)];
seg.end_bias   = [b_a_true(n); b_g_true(n)];
seg.end_wheel  = [wheel_R; wheel_L];
seg.end_counts = [counts_R(n); counts_L(n)];

end
