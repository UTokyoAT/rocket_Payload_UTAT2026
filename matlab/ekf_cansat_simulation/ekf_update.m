function [x_upd, P_upd, y, S] = ekf_update(x, P, z, h_x, H, R)
% EKF_UPDATE  汎用 EKF 更新ステップ（エンコーダ・GNSS 位置で共用）。
%   角度観測の折り返し処理は含まない（COG は ekf_update_cog を使う）。
%
%   z   : 観測ベクトル
%   h_x : 予測状態での観測関数 h(x) の値（呼び出し側で評価して渡す）
%   H   : 観測ヤコビアン
%   R   : 観測ノイズ共分散
%   y, S : イノベーションとその共分散（ログ用）

y = z - h_x;
[x_upd, P_upd, S] = ekf_apply_innovation(x, P, y, H, R);

end
