function [X_est, P_hist] = run_ekf_over_range(X_est, P_hist, k_start, k_end, ...
    A_MEAS, OMEGA_MEAS, COUNTS_R, COUNTS_L, GNSS_FIX, Z_GNSS_N, Z_GNSS_E, ...
    dt, Q, H_enc, R_enc, H_gnss, R_gnss, params, use_encoder)
% RUN_EKF_OVER_RANGE  グローバル観測値履歴のうち [k_start, k_end] 区間だけ
%   EKFループ（予測→エンコーダ更新→GNSS更新、要件書1.7節）を回す。
%   X_est(:,k_start), P_hist(:,:,k_start) は呼び出し前に設定済みであること。
%   run_ekf_simulation.m のメインループと同じ処理を、任意区間に対して
%   繰り返し呼べるように関数化したもの（目的地誘導シナリオで、区間ごとに
%   動的に生成される真値・観測値に対して使う）。
%
%   use_encoder = false にすると、エンコーダ更新ステップを丸ごとスキップし、
%   予測とGNSS更新（1Hz）だけでEKFを回す（エンコーダなし比較用）。

x = X_est(:, k_start);
P = P_hist(:, :, k_start);

for k = k_start:k_end-1
    [x_pred, P_pred] = ekf_predict(x, P, A_MEAS(k), OMEGA_MEAS(k), dt, Q);

    if use_encoder
        v_R = encoder_counts_to_velocity(COUNTS_R(k), COUNTS_R(k+1), dt, params.r);
        v_L = encoder_counts_to_velocity(COUNTS_L(k), COUNTS_L(k+1), dt, params.r);
        z_enc = [v_R; v_L];

        omega_k1 = OMEGA_MEAS(k+1);
        h_enc = [x_pred(3) + (params.L/2)*(omega_k1 - x_pred(6));
                 x_pred(3) - (params.L/2)*(omega_k1 - x_pred(6))];

        [x_upd, P_upd] = ekf_update(x_pred, P_pred, z_enc, h_enc, H_enc, R_enc);
    else
        x_upd = x_pred;
        P_upd = P_pred;
    end

    if GNSS_FIX(k+1)
        z_gnss = [Z_GNSS_N(k+1); Z_GNSS_E(k+1)];
        h_gnss = [x_upd(1); x_upd(2)];
        [x_upd, P_upd] = ekf_update(x_upd, P_upd, z_gnss, h_gnss, H_gnss, R_gnss);
    end

    x = x_upd;
    P = P_upd;
    X_est(:, k+1) = x;
    P_hist(:, :, k+1) = P;
end

end
