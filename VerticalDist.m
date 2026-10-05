function [Rz, dRz] = VerticalDist(posUAV, posHpad, Vp, gamma)
%Rz  =  posHpad(3) - posUAV(3);
Rz  =  posUAV(3) - posHpad(3);
dRz =  -Vp * sin(gamma);
end
