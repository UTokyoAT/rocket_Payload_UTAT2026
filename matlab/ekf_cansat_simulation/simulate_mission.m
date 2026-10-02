function [log, cal, result] = simulate_mission(params)
% SIMULATE_MISSION  キャリブレーション → PID 誘導 → 急停止 までの閉ループシミュレーション（要件書2章）。
%
%   毎ステップ（周期 dt）要件書2.3節の順で処理する：
%     1. 制御則（Phase 3 は方位 PID、それ以外は固定指令）
%     2. 運動モデル更新（k → k+1）
%     3. 疑似観測生成（IMU・エンコーダ・GNSS fix は到着時のみ）
%     4. EKF：予測 → エンコーダ更新 → GNSS 位置更新 → COG 更新（Phase 3, 4 のみ）
%     5. フェーズ遷移・停止判定・ログ記録
%
%   呼び出し側で rng(params.rng_seed) を先に実行しておくこと。
%
%   log    : 各ステップ（時刻 t = k·dt, k = 1..N）の記録。EKF 未起動区間の推定値は NaN
%   cal    : キャリブレーション結果（calibrate_initial_state の出力）
%   result : 停止時刻・停止位置・成否など

dt = params.dt;
n_gnss = round(params.gnss_period / dt);
n_cal  = round(params.t_cal / dt);
n_settle = round(params.t_settle / dt);
ekf = ekf_setup(params);

%% ログ領域の確保
N_MAX = ceil((2*params.N_avg*params.gnss_period + params.t_cal + params.t_max + params.t_post) / dt) + 10;
log = init_log(N_MAX);

%% 初期状態
model = struct('p_N', 0, 'p_E', 0, 'theta', params.theta_model0, ...
               'v', 0, 'v_prev', 0, 'omega', 0, 'v_R', 0, 'v_L', 0);
sens  = struct('wheel_R', 0, 'wheel_L', 0, 'counts_R', 0, 'counts_L', 0);
ctrl  = struct('I', 0, 'sat_prev', false, 'omega_cmd_prev', 0);
stopper = struct('count', 0);

phase = 0;
phase_steps = 0;           % 現フェーズに入ってからのステップ数
gnss0 = zeros(2, 0); gnss2 = zeros(2, 0);
a_static = []; w_static = [];
omega_last = 0;            % 制御則が使う最新のジャイロ生値
x = []; P = [];
cal = [];
result = struct('stopped', false, 't_ekf_start', NaN, 't_stop', NaN, ...
                'p_stop_est', [NaN; NaN], 'timeout', false);

k = 0;
while true
    k = k + 1;
    phase_steps = phase_steps + 1;
    log.phase(k) = phase;

    %% 1. 制御則
    pid = struct('d_hat', NaN, 'e', NaN, 'sat', false);
    whl = struct('sat', false);
    omega_cmd = NaN;
    switch phase
        case 1
            v_R_cmd = params.v_cal; v_L_cmd = params.v_cal;
        case 3
            [omega_cmd, ctrl, pid] = heading_pid(x, omega_last, ctrl, params);
            [v_R_cmd, v_L_cmd, whl] = wheel_speed_command(omega_cmd, pid.e, params);
        otherwise   % Phase 0, 2, 4：静止
            v_R_cmd = 0; v_L_cmd = 0;
    end

    %% 2. 運動モデル更新
    model = motion_model_step(model, v_R_cmd, v_L_cmd, params);

    %% 3. 疑似観測生成
    gnss_arrived = mod(k, n_gnss) == 0;
    [meas, sens] = generate_measurements(model, sens, gnss_arrived, params);
    omega_last = meas.omega_meas;

    %% 4. EKF（Phase 3, 4）
    if phase >= 3
        [x, P] = ekf_predict(x, P, meas.a_meas, meas.omega_meas, dt, ekf.Q);

        % エンコーダ更新（毎ステップ）
        z_enc = [encoder_counts_to_velocity(meas.counts_R_prev, meas.counts_R, params);
                 encoder_counts_to_velocity(meas.counts_L_prev, meas.counts_L, params)];
        w_corr = meas.omega_meas - x(6);
        h_enc = [x(3) - params.L/2 * w_corr;
                 x(3) + params.L/2 * w_corr];
        [x, P, y, S] = ekf_update(x, P, z_enc, h_enc, ekf.H_enc, ekf.R_enc);
        log.y_enc(:,k) = y; log.S_enc(:,k) = diag(S);

        if meas.gnss
            % GNSS 位置更新（予測を挟まず逐次更新）
            z_gnss = [meas.z_N; meas.z_E];
            [x, P, y, S] = ekf_update(x, P, z_gnss, x(1:2), ekf.H_gnss, ekf.R_gnss);
            log.y_gnss(:,k) = y; log.S_gnss(:,k) = diag(S);

            % COG 更新（同一 fix、SOG ゲーティング）
            if cog_update_enabled(meas.sog, params)
                [x, P, y, S] = ekf_update_cog(x, P, meas.cog, ekf.H_cog, ekf.R_cog);
                log.y_cog(k) = y; log.S_cog(k) = S;
                log.cog_applied(k) = true;
            else
                log.cog_gated(k) = true;
            end
        end
    end

    %% 5. フェーズ遷移・停止判定
    next_phase = phase;
    switch phase
        case 0   % 出発地点の測位
            a_static(end+1) = meas.a_meas; w_static(end+1) = meas.omega_meas; %#ok<AGROW>
            if meas.gnss, gnss0(:,end+1) = [meas.z_N; meas.z_E]; end %#ok<AGROW>
            if size(gnss0, 2) >= params.N_avg, next_phase = 1; end
        case 1   % キャリブレーション直進
            if phase_steps >= n_cal, next_phase = 2; end
        case 2   % 初期地点の測位
            if phase_steps > n_settle
                a_static(end+1) = meas.a_meas; w_static(end+1) = meas.omega_meas; %#ok<AGROW>
            end
            if meas.gnss, gnss2(:,end+1) = [meas.z_N; meas.z_E]; end %#ok<AGROW>
            if size(gnss2, 2) >= params.N_avg
                cal = calibrate_initial_state(gnss0, gnss2, a_static, w_static, params);
                x = cal.x0; P = cal.P0;
                result.t_ekf_start = k*dt;
                next_phase = 3;
            end
        case 3   % PID 誘導
            d_hat_now = hypot(params.N_goal - x(1), params.E_goal - x(2));
            [stop_now, stopper] = stop_check(d_hat_now, stopper, params);
            if stop_now
                result.stopped = true;
                result.t_stop = k*dt;
                result.p_stop_est = x(1:2);
                next_phase = 4;
            elseif phase_steps*dt > params.t_max
                result.timeout = true;
            end
        case 4   % 急停止後の記録
    end

    %% ログ記録
    log.t(k) = k*dt;
    log.model(:,k) = [model.p_N; model.p_E; model.v; model.theta];
    log.cmd(:,k) = [v_R_cmd; v_L_cmd];
    log.omega_cmd(k) = omega_cmd;
    log.e(k) = pid.e;
    log.d_hat(k) = pid.d_hat;
    log.sat_omega(k) = pid.sat;
    log.sat_vbase(k) = whl.sat;
    log.a_meas(k) = meas.a_meas;
    log.omega_meas(k) = meas.omega_meas;
    log.gnss(k) = meas.gnss;
    log.z_gnss(:,k) = [meas.z_N; meas.z_E];
    log.cog(k) = meas.cog;
    log.sog(k) = meas.sog;
    if ~isempty(x)
        log.x(:,k) = x;
        log.P(:,:,k) = P;
    end

    %% 終了判定
    if result.timeout
        break;
    end
    if next_phase ~= phase
        phase = next_phase;
        phase_steps = 0;
        if phase == 4 && params.t_post <= 0
            break;
        end
    elseif phase == 4 && phase_steps*dt >= params.t_post
        break;
    end
    if k >= N_MAX
        warning('simulate_mission:bufferFull', 'ログ領域が一杯になったため終了しました。');
        break;
    end
end

log = trim_log(log, k);
result.t_end = k*dt;

end


function log = init_log(N)
log.t = zeros(1, N);
log.phase = zeros(1, N);
log.model = nan(4, N);          % [p_N; p_E; v; θ]（センサ生成用。評価には使わない）
log.cmd = nan(2, N);            % [v_R_cmd; v_L_cmd]
log.omega_cmd = nan(1, N);
log.e = nan(1, N);
log.d_hat = nan(1, N);
log.sat_omega = false(1, N);    % ω_cmd が ±ω_cmd_max で飽和
log.sat_vbase = false(1, N);    % 車輪上限のため v_base を削った
log.a_meas = nan(1, N);
log.omega_meas = nan(1, N);
log.gnss = false(1, N);
log.z_gnss = nan(2, N);
log.cog = nan(1, N);
log.sog = nan(1, N);
log.x = nan(6, N);
log.P = nan(6, 6, N);
log.y_enc = nan(2, N);  log.S_enc = nan(2, N);
log.y_gnss = nan(2, N); log.S_gnss = nan(2, N);
log.y_cog = nan(1, N);  log.S_cog = nan(1, N);
log.cog_applied = false(1, N);
log.cog_gated = false(1, N);
end


function log = trim_log(log, N)
f = fieldnames(log);
for i = 1:numel(f)
    v = log.(f{i});
    if ndims(v) == 3
        log.(f{i}) = v(:,:,1:N);
    else
        log.(f{i}) = v(:,1:N);
    end
end
end
