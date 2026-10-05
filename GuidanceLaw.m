function [dVp_cmd, dalpha_p_cmd, dgamma_cmd, validGuidance] = GuidanceLaw( ...
    Vp, Vt, dVt, alpha_p, gamma, ...
    alpha_t, dalpha_t, ddalpha_t, ...
    Rxy, dRxy, psi, dpsi, S, theta_des, ...
    ka, kb, kc, k1, k2, k3, n, m, validLOS)

% Phase-2 landing guidance. S must use the relative-azimuth surface.
% Angles: rad; rates: rad/s; speeds: m/s; distances: m.
% gamma is positive upward. psi is the LOS azimuth.
% validLOS is the scalar logical output from LOSRate.
% validGuidance reports numerical success, including saturated commands.

% Initialize outputs before every possible early return.
dVp_cmd = 0;
dalpha_p_cmd = 0;
dgamma_cmd = 0;
validGuidance = false;

% Reject the numerical fallback from LOSRate before building A or B.
if ~validLOS
    return;
end

if ~isfinite(Rxy) || Rxy <= 0
    return;
end

if ~isfinite(Vp) || ~isfinite(gamma)
    return;
end

cg = cos(gamma);
sg = sin(gamma);
if Vp <= 1e-6 || abs(cg) <= 1e-6
    return;
end

% The paper uses odd positive n,m with 0 < n < m.
if ~isfinite(n) || ~isfinite(m) || n <= 0 || m <= n
    return;
end
power = n / m;
Sp = sign(S) .* abs(S).^power;

dp = alpha_p - psi;
dt = alpha_t - psi;
td = tan(theta_des);

% Equations (21)-(22).
Ap = [-cos(dp)*cg, Vp*sin(dp)*cg, Vp*cos(dp)*sg; ...
      -sg,          0,             -Vp*cg; ...
      -sin(dp)*cg, -Vp*cos(dp)*cg,  Vp*sin(dp)*sg];
M = [1, 0, 0; td, 1, 0; 0, 0, 1];
A = M * Ap;

% Equation (23), using signed powers of the sliding variables.
dRxy_model = Vt*cos(dt) - Vp*cg*cos(dp);
Fxy = Vp*sin(dp)*cg*dpsi ...
    - dVt*cos(dt) ...
    + Vt*sin(dt)*(dalpha_t - dpsi);

B1 = -k1*Sp(1) + Fxy - ka*dRxy_model;
B2 = td*(Fxy - kb*dRxy_model) - k2*Sp(2) + kb*Vp*sg;
B3 = -Rxy*k3*Sp(3) - kc*Rxy*(dpsi - dalpha_t) ...
    + ddalpha_t*Rxy + dRxy*dpsi ...
    - Vp*cos(dp)*cg*dpsi ...
    - Vt*cos(dt)*(dalpha_t - dpsi) - dVt*sin(dt);
B = [B1; B2; B3];

if ~all(isfinite(A(:))) || ~all(isfinite(B(:)))
    return;
end
if rcond(A) <= 1e-12
    return;
end

Uraw = A \ B;
if ~all(isfinite(Uraw))
    return;
end
dVp = Uraw(1);
dalpha_p = Uraw(2);
dgamma = Uraw(3);

% Algorithm 1 thresholds and symmetric command limits.
M1 = 0.1;
M2 = 0.15;
N1 = 10;
N2 = pi/2;
N3 = pi/2;

if Vp < M1 && dVp < 0
    dVp = 0;
end
if cg < M2 && gamma*dgamma > 0
    dgamma = 0;
end

dVp_cmd = min(N1, max(-N1, dVp));
dalpha_p_cmd = min(N2, max(-N2, dalpha_p));
dgamma_cmd = min(N3, max(-N3, dgamma));
validGuidance = true;
end


