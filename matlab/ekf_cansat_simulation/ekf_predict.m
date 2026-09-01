function [x_pred, P_pred] = ekf_predict(x, P, a_meas, omega_meas, dt, Q)
% EKF_PREDICT  要件書1.2〜1.4節の予測ステップ。
%   x = [p_N; p_E; v; theta; b_a; b_g]

p_N = x(1); p_E = x(2); v = x(3); theta = x(4); b_a = x(5); b_g = x(6);

x_pred = zeros(6,1);
x_pred(1) = p_N + v * cos(theta) * dt;
x_pred(2) = p_E + v * sin(theta) * dt;
x_pred(3) = v + (a_meas - b_a) * dt;
x_pred(4) = theta + (omega_meas - b_g) * dt;
x_pred(5) = b_a;
x_pred(6) = b_g;

F = [1 0  cos(theta)*dt   -v*sin(theta)*dt   0    0;
     0 1  sin(theta)*dt    v*cos(theta)*dt   0    0;
     0 0  1                0                -dt   0;
     0 0  0                1                 0   -dt;
     0 0  0                0                 1    0;
     0 0  0                0                 0    1];

P_pred = F * P * F' + Q;

end
