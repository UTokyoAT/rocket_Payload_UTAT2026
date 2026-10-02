function a = wrap_to_pi(a)
% WRAP_TO_PI  角度を [-π, π) に折り返す（要件書の wrap(·)。Toolbox 非依存）。

a = mod(a + pi, 2*pi) - pi;

end
