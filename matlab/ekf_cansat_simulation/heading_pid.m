function [omega_cmd, ctrl, info] = heading_pid(x_hat, omega_meas, ctrl, params)
% HEADING_PID  EKF 推定値からゴール方位への方位 PID（要件書2.4.2節）。
%
%   x_hat      : EKF 最新推定値
%   omega_meas : 最新のジャイロ生値（D 項はバイアス補正済みジャイロ角速度から取る）
%   ctrl       : 制御器内部状態 struct(I, sat_prev, omega_cmd_prev)
%
%   info : d_hat（推定ゴール距離）, psi_goal, e（方位誤差）, sat（飽和したか）

dN = params.N_goal - x_hat(1);
dE = params.E_goal - x_hat(2);
info.d_hat    = hypot(dN, dE);
info.psi_goal = atan2(dE, dN);
e = wrap_to_pi(info.psi_goal - x_hat(4));   % 真北をまたいでも跳ねないよう必ず wrap
info.e = e;

omega_rate = omega_meas - x_hat(6);

% 積分（アンチワインドアップ：前回飽和中は、飽和を解消する向きの誤差のときだけ積分）
if params.Ki ~= 0
    if ~ctrl.sat_prev || sign(e) ~= sign(ctrl.omega_cmd_prev)
        ctrl.I = ctrl.I + e * params.dt;
    end
    ctrl.I = min(max(ctrl.I, -params.I_max), params.I_max);
end

omega_raw = params.Kp*e + params.Ki*ctrl.I - params.Kd*omega_rate;
omega_cmd = min(max(omega_raw, -params.omega_cmd_max), params.omega_cmd_max);

info.sat = omega_cmd ~= omega_raw;
ctrl.sat_prev = info.sat;
ctrl.omega_cmd_prev = omega_cmd;

end
