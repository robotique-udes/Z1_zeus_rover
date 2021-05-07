function [MR] = computeEntireVehicleMotionResistance(C, V)
% Computes, for the entire vehicle, the motion resistance

% Slope-dependant resistance(s)
MR.gravitationalResistance = C.vehicleMass * C.g * sin(C.inclinationInRadians);

% Slope-independant resistance(s)
MR.soilCompactionResistance = computeSoilCompactionResistance(C, V);
MR.accelerationResistance = C.vehicleMass * C.accelerationNeeded;
MR.windResistance = (1/2) * C.dragCoefficient * C.rhoAir * (C.nominalLinearSpeedNeeded^2) * C.vehiculeFrontArea;

if C.inclinationInRadians == 0
    MR.buldozingResistance =  0.25 * computeBuldozingResistance(C, V);
else
    MR.buldozingResistance =  0.25 * computeBuldozingResistance(C, V);
end

MR.drawbarPullResistance = 0; % Available force calculated instead
MR.rollingResistance = 0; % Negliged

MR.totalMotionResistance = MR.gravitationalResistance + MR.soilCompactionResistance + MR.accelerationResistance + MR.windResistance + MR.buldozingResistance + MR.rollingResistance + MR.drawbarPullResistance;

end

