function [x_upd, P_upd, y, S] = ekf_update_cog(x, P, z_cog, H_cog, R_cog)
% EKF_UPDATE_COG  GNSS COG による方位角の更新（要件書1.7節）。
%   h_cog(x) = θ。イノベーションは (-π, π] に折り返してから使う
%   （z=1°, θ̂=359° のようなケースで −358° にならないようにする）。

y = z_cog - x(4);
y = mod(y + pi, 2*pi) - pi;
[x_upd, P_upd, S] = ekf_apply_innovation(x, P, y, H_cog, R_cog);

end
