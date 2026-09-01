% RUN_GOAL_DIRECTED_SIMULATION  出発地からゴール（約50m先）まで進む目的地誘導シナリオ。
%   EKFは要件書どおりエンコーダ更新+GNSS更新の両方を使う（標準版）。
%   エンコーダを使わない比較版は run_goal_directed_simulation_no_encoder.m を参照。
%
%   外部Toolboxには依存しない。

clear; clc; close all;

params = config_params_goal();
rng(params.rng_seed);

[T, PHASE, gt_state, X_est, P_hist, leg_end_idx, turn_pos_est, turn_theta_est] = simulate_goal_directed(params, true);

N_LEGS_CROP = 5; % 先頭何レグ分を切り抜いたマップも表示するか
plot_results_goal(T, PHASE, gt_state, X_est, P_hist, params, leg_end_idx, N_LEGS_CROP, turn_pos_est, turn_theta_est);
