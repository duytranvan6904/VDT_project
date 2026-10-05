function dRxy = HoriRangeRate(Vt, alpha_t, Vp, gamma, alpha_p, psi)
% Vt, Vp: m/s
% All angles: rad
% dRxy: m/s

dRxy = Vt*cos(alpha_t - psi) ...
     - Vp*cos(gamma)*cos(alpha_p - psi);

end