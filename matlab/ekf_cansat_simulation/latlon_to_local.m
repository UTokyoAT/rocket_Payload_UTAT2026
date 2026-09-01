function [p_N, p_E] = latlon_to_local(lat, lon, lat0, lon0, m_per_deg)
% LATLON_TO_LOCAL  緯度経度[deg]を基準点(lat0,lon0)からの北・東ローカル座標[m]に変換する。
%   要件書1.6節の変換式そのまま:
%     p_N = (lat - lat0) * m_per_deg
%     p_E = (lon - lon0) * m_per_deg * cos(lat0 [rad変換後])

p_N = (lat - lat0) * m_per_deg;
p_E = (lon - lon0) * m_per_deg * cos(deg2rad(lat0));

end
