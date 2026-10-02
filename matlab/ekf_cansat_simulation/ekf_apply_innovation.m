function [x_upd, P_upd, S] = ekf_apply_innovation(x, P, y, H, R)
% EKF_APPLY_INNOVATION  計算済みのイノベーション y からカルマンゲインを求めて状態・共分散を更新する。
%   ekf_update / ekf_update_cog の共通部分。

S = H * P * H' + R;
K = P * H' / S;

x_upd = x + K * y;
P_upd = (eye(size(P)) - K * H) * P;
P_upd = (P_upd + P_upd') / 2;   % 数値誤差による非対称化を防ぐ

end
