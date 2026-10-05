function [Vp, alpha_p, gamma] = VelocityToFlightState(velNED)
% velNED: WorldVelocity, vector 3x1, m/s
% alpha_p and gamma: rad
% gamma positive upward, following the paper

vx = velNED(1);
vy = velNED(2);
vz = velNED(3);

Vxy = hypot(vx, vy);

Vp      = sqrt(vx*vx + vy*vy + vz*vz);
alpha_p = atan2(vy, vx);
gamma   = atan2(-vz, Vxy);
end