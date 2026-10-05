function stop = LandingStop(Rxy, Rz, velUAV, velTarget)
% velUAV and velTarget must use the same coordinate frame
% and have matching dimensions.
stop = 0;
dv = velUAV - velTarget;
vRel = sqrt(dv(1)^2 + dv(2)^2 + dv(3)^2);

% stop = (Rxy <= 0.1) ...
%     && (abs(Rz) <= 0.1) ...
%     && (vRel <= vTol);
if (Rxy <= 0.1) || (abs(Rz) <= 0.1)
    stop = 1;
end
end