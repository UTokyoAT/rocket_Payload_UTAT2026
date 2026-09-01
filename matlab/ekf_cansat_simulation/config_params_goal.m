function params = config_params_goal()
% CONFIG_PARAMS_GOAL  目的地誘導シナリオ用パラメータ。
%   基本パラメータは config_params.m を継承し、目的地誘導に必要な項目だけを追加する。
%   実測値が出たら config_params.m 側を直せば、この関数もそのまま反映される。

params = config_params();

%% ---- 目的地（出発点(0,0)からの相対位置。真北基準・時計回り正の方位角） -----
params.goal_distance_m  = 50;   % [m] 出発点から目的地までの直線距離
params.goal_bearing_deg = 55;   % [deg] 出発点から見た目的地の方位角
params.goal_N = params.goal_distance_m * cosd(params.goal_bearing_deg); % [m]
params.goal_E = params.goal_distance_m * sind(params.goal_bearing_deg); % [m]

%% ---- 誘導ロジック（後日実機の制御周期・許容誤差に合わせて調整） -----------
% 方向転換のたびに、EKF推定位置（実機が使える唯一の情報）から見た目的地への
% 方位角を計算し、その方向へ機体を向け直してから、一定時間（leg_duration_s）
% だけ直進する（距離ベースではなく時間ベース。T_calib_straightと同じ長さ）。
params.leg_duration_s      = params.T_calib_straight; % [s] 1レグ（直進区間）あたりの直進時間
params.arrival_tolerance_m = 1.5;  % [m] これより目的地に近づいたら到達とみなす
params.max_legs            = 30;   % 安全装置（目的地に着かない場合の無限ループ防止）

end
