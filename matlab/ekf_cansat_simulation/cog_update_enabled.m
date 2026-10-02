function tf = cog_update_enabled(sog, params)
% COG_UPDATE_ENABLED  COG 更新の適用可否判定（要件書1.7節 SOG ゲーティング）。
%   SOG が v_threshold を下回るときは COG が数値的に不安定なので更新をスキップする。

tf = ~isnan(sog) && sog >= params.v_threshold;

end
