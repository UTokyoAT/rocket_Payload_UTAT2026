% RUN_GOAL_DIRECTED_SIMULATION_NO_ENCODER  目的地誘導シナリオ（エンコーダ更新なし版）。
%   run_goal_directed_simulation.m と全く同じシナリオ・同じ乱数シードだが、
%   EKFのエンコーダ更新ステップを丸ごと無効化し、予測とGNSS更新（約1Hz）だけで
%   推定する。エンコーダによる速度フィルタリングの有無で推定精度がどれだけ
%   変わるかを比較するための版。
%
%   外部Toolboxには依存しない。

clear; clc; close all;

params = config_params_goal();
rng(params.rng_seed);

[T, PHASE, gt_state, X_est, P_hist, leg_end_idx, turn_pos_est, turn_theta_est] = simulate_goal_directed(params, false);

N_LEGS_CROP = 5; % 先頭何レグ分を切り抜いたマップも表示するか
plot_results_goal(T, PHASE, gt_state, X_est, P_hist, params, leg_end_idx, N_LEGS_CROP, turn_pos_est, turn_theta_est);
