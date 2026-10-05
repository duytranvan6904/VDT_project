function [Rxy, psi] = DistUAVtoHpad(posUAV, posHpad, psiPrev, Rmin)
dx = posHpad(1) - posUAV(1);
dy = posHpad(2) - posUAV(2);

Rxy2 = dx*dx + dy*dy;
Rxy  = sqrt(Rxy2);

valid = Rxy > Rmin;

if valid
    psi  = atan2(dy, dx);
else
    psi  = psiPrev;
end
end

