function [lat, lon] = local_to_latlon(p_N, p_E, lat0, lon0, m_per_deg)
% LOCAL_TO_LATLON  latlon_to_local の逆変換。真の位置から「センサが出力するはずの
%   緯度経度」を作るために、GNSS計測値生成側でのみ使用する。

lat = lat0 + p_N / m_per_deg;
lon = lon0 + p_E / (m_per_deg * cos(deg2rad(lat0)));

end
