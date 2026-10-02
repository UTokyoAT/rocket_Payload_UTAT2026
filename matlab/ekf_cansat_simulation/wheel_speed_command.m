function [v_R_cmd, v_L_cmd, info] = wheel_speed_command(omega_cmd, e, params)
% WHEEL_SPEED_COMMAND  速度計画・差動分配・飽和処理（要件書2.4.3節）。
%   方位誤差が大きいほど減速して旋回を優先し、車輪の最大周速を超える場合は
%   v_base 側を削る（後退はしない。その場旋回は許容）。
%
%   info : v_base（計画値）, v_base_eff（飽和処理後）, sat（v_base を削ったか）

half_L = params.L / 2;

v_base = params.v_cruise * max(0, cos(e));
v_base_eff = min(v_base, params.v_wheel_max - half_L*abs(omega_cmd));
v_base_eff = max(v_base_eff, 0);

% 差動分配（ω_cmd > 0 で右旋回 → 右輪が遅い）
v_R_cmd = v_base_eff - half_L*omega_cmd;
v_L_cmd = v_base_eff + half_L*omega_cmd;

info.v_base     = v_base;
info.v_base_eff = v_base_eff;
info.sat        = v_base_eff < v_base;

end
