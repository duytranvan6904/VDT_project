function [dpsi, valid] = LOSRate( ...
    Vt, alpha_t, Vp, gamma, alpha_p, psi, Rxy, Rmin)

% Speeds: m/s; distances: m
% Angles: rad; dpsi: rad/s
% Rmin must be positive.

valid = (Rxy > Rmin) && (Rmin > 0);

if valid
    % Equation (4)
    dpsi = (Vt*sin(alpha_t - psi) ...
          - Vp*cos(gamma)*sin(alpha_p - psi)) / Rxy;
else
    % Numerical fallback, not the actual LOS angular rate.
    dpsi = 0;
end
end