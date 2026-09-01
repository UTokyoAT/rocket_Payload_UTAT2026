function plot_results_goal(t, phase, gt_state, X_est, P_hist, params, leg_end_idx, n_legs_crop, turn_pos_est, turn_theta_est)
% PLOT_RESULTS_GOAL  目的地誘導シナリオ用の検証プロット。
%   1) 真の軌道 vs EKF推定軌道（p_N-p_E平面）、出発点・目的地・到達許容円つき（全区間）
%   2) 同上を先頭 n_legs_crop レグ分だけに切り抜いたマップ
%   3) 各状態変数の推定誤差の時間変化と、共分散P由来の±3σ範囲
%
%   leg_end_idx    : 1xM、各レグ完了時点のグローバルインデックス（simulate_goal_directed.m出力）
%   n_legs_crop    : 切り抜き図で表示するレグ数（実際に完了したレグ数がこれ未満ならその数まで）
%   turn_pos_est   : 2xM、各レグの方向転換直前のEKF推定位置 [p_N;p_E]
%   turn_theta_est : 1xM、同時点のEKF推定方位角[rad]。軌道図に矢印(→)で重ねて表示する。

%% --- 1) 軌道プロット（全区間） ---
plot_trajectory(gt_state, X_est, params, 1:numel(t), true, turn_pos_est, turn_theta_est, ...
    'Goal-directed Trajectory: Ground Truth vs EKF', ...
    sprintf('Goal-directed Navigation (%.0fm @ bearing %.0f deg) - full run', ...
        params.goal_distance_m, params.goal_bearing_deg));

%% --- 2) 軌道プロット（先頭 n_legs_crop レグだけ切り抜き、ゴールは表示しない） ---
% ゴール・到達許容円まで表示すると軸がそこまで引き伸ばされて肝心の序盤の
% 軌道が見づらくなるため、切り抜き図では軌道そのものだけを拡大表示する。
if ~isempty(leg_end_idx)
    n_shown = min(n_legs_crop, numel(leg_end_idx));
    crop_end = leg_end_idx(n_shown);
    plot_trajectory(gt_state, X_est, params, 1:crop_end, false, ...
        turn_pos_est(:,1:n_shown), turn_theta_est(1:n_shown), ...
        sprintf('Goal-directed Trajectory: First %d legs', n_shown), ...
        sprintf('First %d of %d legs (t=0 to %.1fs)', n_shown, numel(leg_end_idx), t(crop_end)));
else
    warning('plot_results_goal:noLegs', 'No completed legs to crop; skipping cropped map.');
end

%% --- 3) 状態誤差と±3σ ---
err = gt_state - X_est;
state_names = {'p_N [m]', 'p_E [m]', 'v [m/s]', '\theta [rad]', 'b_a [m/s^2]', 'b_g [rad/s]'};

sigma3 = zeros(6, numel(t));
for k = 1:numel(t)
    sigma3(:,k) = 3 * sqrt(diag(P_hist(:,:,k)));
end

figure('Name', 'Goal-directed: EKF Estimation Error vs +/-3 sigma');
for i = 1:6
    subplot(3, 2, i);
    hold on; grid on;
    plot(t, err(i,:), 'b-', 'LineWidth', 1.0, 'DisplayName', 'error');
    plot(t, sigma3(i,:), 'r--', 'LineWidth', 0.8, 'DisplayName', '+3\sigma');
    plot(t, -sigma3(i,:), 'r--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    xlabel('t [s]');
    ylabel(state_names{i});
    title(state_names{i});
    if i == 1
        legend('Location', 'best');
    end
end
sgtitle('State Estimation Error and \pm3\sigma Bounds (Goal-directed scenario)');

end


function plot_trajectory(gt_state, X_est, params, range, show_goal, turn_pos_est, turn_theta_est, fig_name, fig_title)
figure('Name', fig_name);
hold on; grid on; axis equal;
plot(gt_state(2,range), gt_state(1,range), 'k-', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
plot(X_est(2,range), X_est(1,range), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
plot(0, 0, 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Start');

if show_goal
    plot(params.goal_E, params.goal_N, 'g^', 'MarkerFaceColor', 'g', 'MarkerSize', 10, 'DisplayName', 'Goal');
    th_c = linspace(0, 2*pi, 100);
    plot(params.goal_E + params.arrival_tolerance_m*sin(th_c), ...
         params.goal_N + params.arrival_tolerance_m*cos(th_c), ...
         'g:', 'DisplayName', 'Arrival tolerance');
end

if ~isempty(turn_pos_est)
    % 方向転換直前のEKF推定方位角を矢印(→)で表示。長さは表示用の固定値。
    arrow_len = 2; % [m]
    dE = arrow_len * sin(turn_theta_est);
    dN = arrow_len * cos(turn_theta_est);
    quiver(turn_pos_est(2,:), turn_pos_est(1,:), dE, dN, 0, ...
        'b', 'LineWidth', 1.3, 'MaxHeadSize', 1.5, 'DisplayName', 'Heading est. (pre-turn)');
end

xlabel('p_E [m] (East)');
ylabel('p_N [m] (North)');
title(fig_title);
legend('Location', 'best');
end
