function ekf = ekf_setup(params)
% EKF_SETUP  状態に依存しない EKF 行列を初期化時に一度だけ構成する（要件書1.4〜1.7節）。
%   Q = B・Q_w・B^T もここで一度だけ計算し、予測ステップでは再計算しない。

dt = params.dt;

%% プロセスノイズ（1.4節）
ekf.Q_w = diag([params.sigma_a^2, params.sigma_g^2, params.sigma_rw_a^2, params.sigma_rw_g^2]);
ekf.B = [0   0   0        0;
         0   0   0        0;
         dt  0   0        0;
         0   dt  0        0;
         0   0   sqrt(dt) 0;
         0   0   0        sqrt(dt)];
ekf.Q = ekf.B * ekf.Q_w * ekf.B';

%% エンコーダ（1.5節、符号訂正版）
ekf.H_enc = [0 0 1 0 0  params.L/2;
             0 0 1 0 0 -params.L/2];
ekf.R_enc = diag([params.r^2 * 2*params.sigma_theta_R^2 / dt^2, ...
                  params.r^2 * 2*params.sigma_theta_L^2 / dt^2]);

%% GNSS 位置（1.6節）
ekf.H_gnss = [1 0 0 0 0 0;
              0 1 0 0 0 0];
ekf.R_gnss = diag([params.sigma_N^2, params.sigma_E^2]);

%% GNSS COG（1.7節）
ekf.H_cog = [0 0 0 1 0 0];
ekf.R_cog = params.sigma_cog^2;

end
