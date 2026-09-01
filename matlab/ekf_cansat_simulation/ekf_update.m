function [x_upd, P_upd] = ekf_update(x, P, z, h_x, H, R)
% EKF_UPDATE  汎用EKF更新ステップ（エンコーダ・GNSS共通、要件書4章の要求どおり
%   観測モデルに依存しない汎用形にしてある）。
%
%   z   : 観測ベクトル
%   h_x : 現在の予測状態xでの観測関数h(x)の値（呼び出し側で評価して渡す）
%   H   : 観測ヤコビアン
%   R   : 観測ノイズ共分散

y = z - h_x;                 % innovation
S = H * P * H' + R;
K = P * H' / S;

x_upd = x + K * y;
P_upd = (eye(size(P)) - K * H) * P;

end
