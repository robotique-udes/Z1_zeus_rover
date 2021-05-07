function [Total] = computeResistance(inclination, linearSpeed)
[C, V] = loadConstants();
C.inclination = inclination; 
C.inclinationInRadians = (C.inclination/360)*2*pi;
C.nominalLinearSpeedNeeded = linearSpeed;
V.revPerMinute = (C.nominalLinearSpeedNeeded / V.circonferenceInMeters) * 60;
V.wheelRadPerSecond = (V.revPerMinute * 2 * pi) / 60;

MR = computeEntireVehicleMotionResistance(C, V);
Total = MR.totalMotionResistance;

end

