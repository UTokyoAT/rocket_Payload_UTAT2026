% RUN_SIMULATION  CanSat 2輪誘導 EKF + 方位PID シミュレーションのメインスクリプト。
%   要件書「EKF設計及びシミュレーション方法.md」に基づく。追加 Toolbox 不使用。
%   パラメータはすべて config_params.m で変更する。

clear; clc; close all;

params = config_params();
rng(params.rng_seed);

[log, cal, result] = simulate_mission(params);

%% 5.5 キャリブレーション結果
fprintf('==== キャリブレーション結果 (dt = %.3f s) ====\n', params.dt);
fprintf('P_start = (N %.2f, E %.2f) m,  P_init = (N %.2f, E %.2f) m\n', ...
    cal.P_start(1), cal.P_start(2), cal.P_init(1), cal.P_init(2));
fprintf('D = %.2f m,  theta0 = %.1f deg\n', cal.D, rad2deg(cal.theta0));
fprintf('b_a0 = %.4f m/s^2,  b_g0 = %.3f deg/s  (静止サンプル %d 点)\n', ...
    cal.b_a0, rad2deg(cal.b_g0), cal.n_static);
fprintf('x0 = [%.2f, %.2f, %.3f, %.4f rad, %.4f, %.5f rad/s]\n', cal.x0);
fprintf('sqrt(diag(P0)) = [%.3f m, %.3f m, %.3f m/s, %.1f deg, %.4f m/s^2, %.4f deg/s]\n', ...
    sqrt(cal.P0(1,1)), sqrt(cal.P0(2,2)), sqrt(cal.P0(3,3)), rad2deg(sqrt(cal.P0(4,4))), ...
    sqrt(cal.P0(5,5)), rad2deg(sqrt(cal.P0(6,6))));
disp('P0 ='); disp(cal.P0);

%% 5.4 停止結果
fprintf('==== 誘導結果 ====\n');
fprintf('EKF 起動: t = %.1f s\n', result.t_ekf_start);
if result.stopped
    fprintf('急停止: t = %.1f s (誘導 %.1f s),  推定停止位置 = (N %.2f, E %.2f) m,  推定ゴール距離 %.2f m\n', ...
        result.t_stop, result.t_stop - result.t_ekf_start, result.p_stop_est(1), result.p_stop_est(2), ...
        hypot(params.N_goal - result.p_stop_est(1), params.E_goal - result.p_stop_est(2)));
elseif result.timeout
    fprintf('誘導失敗: t_max = %.0f s 以内に停止条件を満たさなかった\n', params.t_max);
end
fprintf('COG 更新: 適用 %d 回 / ゲート(SOG<%.2f m/s) %d 回\n', ...
    nnz(log.cog_applied), params.v_threshold, nnz(log.cog_gated));

plot_results(log, cal, result, params);
