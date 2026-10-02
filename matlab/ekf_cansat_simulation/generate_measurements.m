function [meas, sens] = generate_measurements(model, sens, gnss_arrived, params)
% GENERATE_MEASUREMENTS  運動モデルの k→k+1 の動きから疑似観測値を生成する（要件書2.6節）。
%
%   model        : motion_model_step 実行後の運動モデル
%   sens         : センサ内部状態 struct(wheel_R, wheel_L, counts_R, counts_L)
%                  wheel_* は連続車輪回転角[rad]、counts_* は直前の量子化カウント
%   gnss_arrived : このステップで GNSS fix が届くか
%
%   meas : a_meas, omega_meas          IMU（k→k+1 区間の値）
%          counts_R, counts_L          今回のエンコーダカウント（量子化済み）
%          counts_R_prev, counts_L_prev 前回のエンコーダカウント
%          gnss (logical), z_N, z_E, cog, sog   GNSS（未到着なら NaN）

dt = params.dt;

%% IMU（バイアスは一定値）
meas.a_meas     = (model.v - model.v_prev)/dt + params.b_a_const + params.sigma_a*randn();
meas.omega_meas = model.omega + params.b_g_const + params.sigma_g*randn();

%% エンコーダ：θ_wheel = ∫(v_i/r)dt に角度ノイズを加え 2π/N で量子化
sens.wheel_R = sens.wheel_R + model.v_R/params.r * dt;
sens.wheel_L = sens.wheel_L + model.v_L/params.r * dt;
res = 2*pi / params.enc_counts_per_rev;
meas.counts_R_prev = sens.counts_R;
meas.counts_L_prev = sens.counts_L;
sens.counts_R = round((sens.wheel_R + params.sigma_theta_R*randn()) / res);
sens.counts_L = round((sens.wheel_L + params.sigma_theta_L*randn()) / res);
meas.counts_R = sens.counts_R;
meas.counts_L = sens.counts_L;

%% GNSS（位置・COG・SOG は同一 fix）
meas.gnss = gnss_arrived;
meas.z_N = NaN; meas.z_E = NaN; meas.cog = NaN; meas.sog = NaN;
if gnss_arrived
    % 位置：真の位置 → 緯度経度 → ノイズ付加 → ローカル座標（1.6節の変換式）
    [lat, lon] = local_to_latlon(model.p_N, model.p_E, params.lat0, params.lon0, params.EARTH_M_PER_DEG);
    lat = lat + params.sigma_N*randn() / params.EARTH_M_PER_DEG;
    lon = lon + params.sigma_E*randn() / (params.EARTH_M_PER_DEG*cos(deg2rad(params.lat0)));
    [meas.z_N, meas.z_E] = latlon_to_local(lat, lon, params.lat0, params.lon0, params.EARTH_M_PER_DEG);

    % SOG / COG：低速ほど COG が悪化する
    meas.sog = abs(model.v + params.sigma_sog*randn());
    sigma_cog_eff = sqrt(params.sigma_cog^2 + (params.sigma_sog / max(model.v, params.cog_v_eps))^2);
    meas.cog = mod(model.theta + sigma_cog_eff*randn(), 2*pi);  % [0, 2π)（NMEA 同様の表現）
end

end
