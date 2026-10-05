function S = SlidingSurface(Rxy, dRxy, Rz, dRz, ...
    psi, dpsi, alpha_t, dalpha_t, ...
    theta_des, zeta_des, ka, kb, kc)
% Distances: m; range rates: m/s
% Angles: rad; angular rates: rad/s
% S: 3x1 vector
%
% psi: LOS azimuth, not UAV body yaw
% theta_des: desired LOS elevation
% zeta_des: desired LOS azimuth relative to target heading

% Equation (17): horizontal range
S1 = dRxy + ka*Rxy;

% Equation (17): vertical range and elevation constraint
td = tan(theta_des);

S2 = dRz + td*dRxy ...
   + kb*(Rz + td*Rxy);

% Equation (17): relative azimuth constraint
%e_psi = psi - alpha_t - zeta_des;
e_raw = psi - alpha_t - zeta_des;
e_psi = atan2(sin(e_raw), cos(e_raw));
S3 = (dpsi - dalpha_t) + kc*e_psi;

% Input to guidanceLaw
S = [S1; S2; S3];

end
