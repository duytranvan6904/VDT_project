function V_cmd  = VelComponents(Vp_cmd, alpha_cmd, gamma_cmd)
% Vp: m/s
% gamma, alpha_p: rad
% Outputs: velocity in NED, m/s

Vx =  Vp_cmd*cos(gamma_cmd)*cos(alpha_cmd);
Vy =  Vp_cmd*cos(gamma_cmd)*sin(alpha_cmd);
Vz =  -Vp_cmd*sin(gamma_cmd);

V_cmd = [Vx Vy Vz];
end
