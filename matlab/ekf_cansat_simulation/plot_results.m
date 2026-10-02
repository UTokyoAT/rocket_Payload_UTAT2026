function plot_results(log, cal, result, params)
% PLOT_RESULTS  要件書5章の観察項目をプロットする（真値との比較は行わない）。
%   図1 推定軌道（5.1）  図2 各状態 ±3σ（5.2）  図3 イノベーション（5.3）  図4 誘導挙動（5.4）

t = log.t;
ekf_on = ~isnan(log.x(1,:));
phase_t = phase_boundaries(log);

plot_trajectory(log, cal, result, params, ekf_on);
plot_states(log, ekf_on, phase_t);
plot_innovations(log, phase_t);
plot_guidance(log, result, params, phase_t);

end


%% ======================================================================
function pt = phase_boundaries(log)
% 各フェーズの開始時刻（Phase 1〜4）
pt = nan(1, 4);
for p = 1:4
    i = find(log.phase == p, 1);
    if ~isempty(i), pt(p) = log.t(i) - (log.t(2) - log.t(1)); end
end
end


function mark_phases(pt)
labels = {'P1 cal', 'P2', 'P3 PID', 'P4 stop'};
yl = ylim;
for p = 1:numel(pt)
    if ~isnan(pt(p))
        xline(pt(p), 'k:', labels{p}, 'HandleVisibility', 'off', ...
            'LabelVerticalAlignment', 'bottom', 'FontSize', 7);
    end
end
ylim(yl);
end


%% ---- 5.1 推定軌道 ------------------------------------------------------
function plot_trajectory(log, cal, result, params, ekf_on)
figure('Name', '5.1 Estimated trajectory');
hold on; grid on; axis equal;

g_cal = log.gnss & log.phase <= 2;
g_ekf = log.gnss & log.phase >= 3;
plot(log.z_gnss(2,g_cal), log.z_gnss(1,g_cal), '.', 'Color', [0.6 0.6 0.6], ...
    'MarkerSize', 8, 'DisplayName', 'GNSS (calibration)');
plot(log.z_gnss(2,g_ekf), log.z_gnss(1,g_ekf), '.', 'Color', [0.2 0.5 0.9], ...
    'MarkerSize', 8, 'DisplayName', 'GNSS raw');
plot(log.x(2,ekf_on), log.x(1,ekf_on), 'r-', 'LineWidth', 1.5, 'DisplayName', 'EKF estimate');

% 1σ 誤差楕円（約10秒おき）
idx = find(ekf_on);
step = max(1, round(10 / params.dt));
th = linspace(0, 2*pi, 60);
first = true;
for i = idx(1:step:end)
    Pp = log.P(1:2,1:2,i);
    [V, Dg] = eig((Pp + Pp')/2);
    c = V * sqrt(max(Dg, 0)) * [cos(th); sin(th)];
    h = plot(log.x(2,i) + c(2,:), log.x(1,i) + c(1,:), 'm-', 'LineWidth', 0.6);
    if first, h.DisplayName = '1\sigma ellipse (10 s)'; first = false;
    else, h.HandleVisibility = 'off'; end
end

plot(cal.P_start(2), cal.P_start(1), 'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'P_{start}');
plot(cal.P_init(2), cal.P_init(1), 'bd', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'P_{init}');
plot(params.E_goal, params.N_goal, 'g^', 'MarkerFaceColor', 'g', 'MarkerSize', 10, 'DisplayName', 'Goal');
plot(params.E_goal + params.d_stop*sin(th), params.N_goal + params.d_stop*cos(th), 'g--', ...
    'DisplayName', sprintf('Stop circle (%.0f m)', params.d_stop));
if result.stopped
    plot(result.p_stop_est(2), result.p_stop_est(1), 'rp', 'MarkerFaceColor', 'r', ...
        'MarkerSize', 12, 'DisplayName', 'Stop (est.)');
end
xlabel('p_E [m] (East)'); ylabel('p_N [m] (North)');
title('Estimated trajectory (p_N – p_E)');
legend('Location', 'bestoutside');
end


%% ---- 5.2 各状態 ±3σ ----------------------------------------------------
function plot_states(log, ekf_on, pt)
t = log.t;
sig3 = nan(6, numel(t));
for i = find(ekf_on)
    sig3(:,i) = 3*sqrt(diag(log.P(:,:,i)));
end
scale = [1 1 1 180/pi 1 180/pi];
names = {'p_N [m]', 'p_E [m]', 'v [m/s]', '\theta [deg]', 'b_a [m/s^2]', 'b_g [deg/s]'};

figure('Name', '5.2 States and +/-3 sigma');
for s = 1:6
    subplot(3, 2, s); hold on; grid on;
    xs = log.x(s,:) * scale(s);
    ss = sig3(s,:) * scale(s);
    fill_band(t, xs - ss, xs + ss, [1 0.8 0.8]);
    plot(t, xs, 'r-', 'LineWidth', 1.0, 'DisplayName', 'estimate');
    if s == 4
        % COG（θ̂ の周りに展開して表示）と、COG 更新の適用／ゲートを区別
        cog_unw = (log.x(4,:) + wrap_to_pi(log.cog - log.x(4,:))) * scale(s);
        plot(t(log.cog_applied), cog_unw(log.cog_applied), 'go', 'MarkerSize', 4, ...
            'DisplayName', 'COG (applied)');
        plot(t(log.cog_gated), cog_unw(log.cog_gated), 'x', 'Color', [0.5 0.5 0.5], ...
            'MarkerSize', 4, 'DisplayName', 'COG (gated, SOG<v_{th})');
        legend('Location', 'best');
    end
    ylabel(names{s}); xlabel('t [s]');
    title([names{s} '  (\pm3\sigma band)']);
    mark_phases(pt);
end
sgtitle('EKF state estimates with \pm3\sigma');
end


%% ---- 5.3 イノベーション ------------------------------------------------
function plot_innovations(log, pt)
t = log.t;
sets = { log.y_enc(1,:),  log.S_enc(1,:),  'Encoder v_R [m/s]';
         log.y_enc(2,:),  log.S_enc(2,:),  'Encoder v_L [m/s]';
         log.y_gnss(1,:), log.S_gnss(1,:), 'GNSS p_N [m]';
         log.y_gnss(2,:), log.S_gnss(2,:), 'GNSS p_E [m]';
         log.y_cog*180/pi, log.S_cog*(180/pi)^2, 'COG [deg]' };

figure('Name', '5.3 Innovations');
for j = 1:size(sets, 1)
    subplot(size(sets, 1), 1, j); hold on; grid on;
    y = sets{j,1}; b = 3*sqrt(sets{j,2});
    m = ~isnan(y);
    plot(t(m), b(m), 'k--', 'DisplayName', '\pm3\surd S');
    plot(t(m), -b(m), 'k--', 'HandleVisibility', 'off');
    if nnz(m) > 200
        plot(t(m), y(m), 'b-', 'DisplayName', 'y');
    else
        plot(t(m), y(m), 'b.-', 'MarkerSize', 10, 'DisplayName', 'y');
    end
    inside = mean(abs(y(m)) <= b(m)) * 100;
    title(sprintf('%s   (inside \\pm3\\surdS: %.1f %%, mean y = %.3g, n = %d)', ...
        sets{j,3}, inside, mean(y(m)), nnz(m)));
    ylabel(sets{j,3});
    if j == 1, legend('Location', 'best'); end
    mark_phases(pt);
end
xlabel('t [s]');
sgtitle('Innovations y and \pm3\surd S');
end


%% ---- 5.4 誘導の挙動 ----------------------------------------------------
function plot_guidance(log, result, params, pt)
t = log.t;
figure('Name', '5.4 Guidance');

subplot(4,1,1); hold on; grid on;
plot(t, log.d_hat, 'b-', 'DisplayName', 'd_{est}');
yline(params.d_stop, 'g--', 'd_{stop}', 'DisplayName', 'd_{stop}');
if result.stopped
    xline(result.t_stop, 'r-', 'stop', 'DisplayName', 'stop');
end
ylabel('d_{est} [m]'); title('Estimated distance to goal'); mark_phases(pt);

subplot(4,1,2); hold on; grid on;
plot(t, rad2deg(log.e), 'b-');
ylabel('e [deg]'); title('Heading error e = wrap(\psi_{goal} - \theta_{est})'); mark_phases(pt);

subplot(4,1,3); hold on; grid on;
plot(t, log.omega_cmd, 'b-', 'DisplayName', '\omega_{cmd}');
plot(t(log.sat_omega), log.omega_cmd(log.sat_omega), 'r.', 'MarkerSize', 8, 'DisplayName', 'saturated');
yline(params.omega_cmd_max, 'k:', 'HandleVisibility', 'off');
yline(-params.omega_cmd_max, 'k:', 'HandleVisibility', 'off');
ylabel('\omega_{cmd} [rad/s]'); title('Turn-rate command'); legend('Location', 'best'); mark_phases(pt);

subplot(4,1,4); hold on; grid on;
plot(t, log.cmd(1,:), 'r-', 'DisplayName', 'v_{R,cmd}');
plot(t, log.cmd(2,:), 'b-', 'DisplayName', 'v_{L,cmd}');
sat = log.sat_vbase;
plot(t(sat), zeros(1, nnz(sat)), 'm.', 'MarkerSize', 6, 'DisplayName', 'v_{base} cut (wheel limit)');
yline(params.v_wheel_max, 'k:', 'v_{wheel,max}', 'HandleVisibility', 'off');
ylabel('[m/s]'); xlabel('t [s]'); title('Wheel speed commands'); legend('Location', 'best'); mark_phases(pt);
end


function fill_band(t, lo, hi, color)
m = ~isnan(lo) & ~isnan(hi);
if ~any(m), return; end
% 連続区間ごとに塗る
d = diff([0 m 0]);
s = find(d == 1); e = find(d == -1) - 1;
for i = 1:numel(s)
    r = s(i):e(i);
    fill([t(r) fliplr(t(r))], [lo(r) fliplr(hi(r))], color, ...
        'EdgeColor', 'none', 'FaceAlpha', 0.6, 'HandleVisibility', 'off');
end
end
