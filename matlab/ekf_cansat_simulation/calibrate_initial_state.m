function cal = calibrate_initial_state(gnss0, gnss2, a_static, w_static, params)
% CALIBRATE_INITIAL_STATE  キャリブレーション（Phase 0〜2）の結果から x0, P0 を求める（要件書2.2節）。
%
%   gnss0    : 2xN_avg Phase 0 の GNSS 測位 [N; E]
%   gnss2    : 2xN_avg Phase 2 の GNSS 測位 [N; E]
%   a_static : 静止区間（Phase 0 + Phase 2）の加速度計生値
%   w_static : 静止区間（Phase 0 + Phase 2）のジャイロ生値
%
%   cal : P_start, P_init, D, theta0, b_a0, b_g0, sigma_ba0, sigma_bg0, x0, P0

N_avg = params.N_avg;

%% 測位平均
cal.P_start = mean(gnss0, 2);
cal.P_init  = mean(gnss2, 2);
dP = cal.P_init - cal.P_start;
cal.D      = hypot(dP(1), dP(2));
cal.theta0 = atan2(dP(2), dP(1));   % コンパス規約：atan2(東, 北)

%% バイアス推定（機体は水平と仮定し、重力の前後軸成分は無視）
n = numel(a_static);
cal.b_a0 = mean(a_static);
cal.b_g0 = mean(w_static);
se_a = std(a_static) / sqrt(n);
se_g = std(w_static) / sqrt(n);
cal.sigma_ba0 = pick(params.sigma_ba0, params.bias_se_margin * se_a);
cal.sigma_bg0 = pick(params.sigma_bg0, params.bias_se_margin * se_g);
cal.n_static = n;

%% x0, P0
sigma_p2 = (params.sigma_N^2 + params.sigma_E^2) / 2;
P0_theta = params.alpha * 2*sigma_p2 / (N_avg * cal.D^2);
P0_theta = min(P0_theta, params.P0_theta_max);

cal.x0 = [cal.P_init(1); cal.P_init(2); 0; cal.theta0; cal.b_a0; cal.b_g0];
cal.P0 = diag([params.alpha * params.sigma_N^2 / N_avg, ...
               params.alpha * params.sigma_E^2 / N_avg, ...
               params.sigma_v0^2, ...
               P0_theta, ...
               cal.sigma_ba0^2, ...
               cal.sigma_bg0^2]);

end


function v = pick(fixed, auto)
% config で固定値が与えられていればそれを、空なら自動算出値を使う
if isempty(fixed)
    v = auto;
else
    v = fixed;
end
end
