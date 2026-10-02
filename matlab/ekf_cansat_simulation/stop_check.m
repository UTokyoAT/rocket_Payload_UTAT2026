function [stop_now, stopper] = stop_check(d_hat, stopper, params)
% STOP_CHECK  急停止判定（要件書2.4.4節）。
%   d̂ ≤ d_stop が「連続」N_stop ループ成立したら停止。1回でも外れたらカウンタを0に戻す。
%
%   stopper : struct(count)

if d_hat <= params.d_stop
    stopper.count = stopper.count + 1;
else
    stopper.count = 0;
end
stop_now = stopper.count >= params.N_stop;

end
