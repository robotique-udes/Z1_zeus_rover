function [Rb] = computeBuldozingResistance(C,V)
% TO FINISH !!!
% Second term seems to high. Don't know why...


% IMPORTANT: Angle in radians is used everywhere except here
% Do not forget to use radians in other functions

theta = C.soilFrictionAngleInRadians; % In radians

z = computeSinkageZ(C, V);
epsi = C.wheelFlexibleDeformation * V.wheelsDiameter;
L = 2 * sqrt((V.wheelsDiameter - epsi)*epsi);
A = (pi/4)*V.wheelWidth * L;
alpha = 1/(cos(1 - (2*z*epsi)/(V.wheelsDiameter)));

l0 = z * tan((pi/4) - (theta/2))^2; % OK RADIANS

% Results in radians or degrees? 
Ky = ((2 * C.ny / tan(theta)) + 1) * (cos(theta)^2);
Kc = (C.nc - tan(theta)) * (cos(theta)^2);

firstTerm =  (V.wheelWidth * sin(alpha + theta))/(2 * sin(alpha) * cos(theta));
secondTerm = (2 * z * C.soilInternalCohesionC * Kc) + (C.g * C.soilVolumicMassY * (z^2) * Ky); % G ok?
thirdTerm = (((l0^3)*C.soilVolumicMassY)/3)*((pi/2) - theta);
fourthTerm = C.soilInternalCohesionC * (l0^2) * (1 + tan((pi/4) + (theta/2)));

Rb = (firstTerm * secondTerm) + (thirdTerm + fourthTerm);
        
end

