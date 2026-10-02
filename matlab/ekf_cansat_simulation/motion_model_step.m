function model = motion_model_step(model, v_R_cmd, v_L_cmd, params)
% MOTION_MODEL_STEP  センサ疑似観測生成専用の理想運動モデルを k→k+1 に進める（要件書2.5節）。
%   指令どおりの車輪速度が即座に出る（応答遅れ・左右差なし）。評価・比較には使わない。
%
%   model : struct(p_N, p_E, theta, v, omega, v_R, v_L)
%           v, omega, v_R, v_L はこのステップ（k→k+1）の区間で実現した値。
%           v_prev は直前区間の速度（IMU 加速度の生成に使う）。

vmax = params.v_wheel_max;
v_R = min(max(v_R_cmd, -vmax), vmax);
v_L = min(max(v_L_cmd, -vmax), vmax);

v     = (v_R + v_L) / 2;
omega = (v_L - v_R) / params.L;   % 時計回り（右旋回）正

model.v_prev = model.v;
model.p_N   = model.p_N + v * cos(model.theta) * params.dt;
model.p_E   = model.p_E + v * sin(model.theta) * params.dt;
model.theta = model.theta + omega * params.dt;
model.v     = v;
model.omega = omega;
model.v_R   = v_R;
model.v_L   = v_L;

end
