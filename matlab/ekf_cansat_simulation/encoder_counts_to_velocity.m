function v = encoder_counts_to_velocity(counts_prev, counts_curr, dt, r)
% ENCODER_COUNTS_TO_VELOCITY  AS5600 RAW ANGLEレジスタ値(12bit, 0-4095カウント =
%   0-360°)の連続する2サンプルから、ホイール周速[m/s]を算出する（要件書1.5節）。
%
%   レジスタは0-4095でラップアラウンドするため、差分を[-2048,2048)カウントの
%   範囲に折り返してから角度差に変換する（1周期内の回転のみを想定）。

COUNTS_PER_REV = 4096;

d = counts_curr - counts_prev;
if d > COUNTS_PER_REV/2
    d = d - COUNTS_PER_REV;
elseif d < -COUNTS_PER_REV/2
    d = d + COUNTS_PER_REV;
end

dtheta = d / COUNTS_PER_REV * 2*pi; % [rad]
v = dtheta / dt * r;

end
