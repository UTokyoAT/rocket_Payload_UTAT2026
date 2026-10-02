function v = encoder_counts_to_velocity(counts_prev, counts_curr, params)
% ENCODER_COUNTS_TO_VELOCITY  量子化された車輪回転角（カウント）の今と1個前の差から
%   車輪周速[m/s]を求める（要件書1.5節）。
%     v_wheel = (θ_wheel(k) − θ_wheel(k−1)) / Δt × r,   θ_wheel = counts × 2π/N

dtheta = (counts_curr - counts_prev) * 2*pi / params.enc_counts_per_rev;
v = dtheta / params.dt * params.r;

end
