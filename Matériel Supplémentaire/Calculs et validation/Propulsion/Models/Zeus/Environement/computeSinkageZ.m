function [wheelSinkageZ] = computeSinkageZ(C, V)
% Valid only if the wheel is considered rigid compared to the soil

% sinkageDeformationModulusK = computeSinkageDeformationModulusK(C, V);
% numerator = 3 * C.vehicleMass * cos(C.inclinationInRadians) / V.wheelsNumber;
% denominator = (3 - V.n) * sinkageDeformationModulusK * V.wheelWidth * sqrt(V.wheelsDiameter);
% wheelSinkageZ = (numerator/denominator)^(2/((2 * V.n) + 1));

sinkageDeformationModulusK = computeSinkageDeformationModulusK(C, V);
numerator = 3 * C.vehicleMass * cos(C.inclinationInRadians) / V.wheelsNumber;
denominator = (3 - V.N) * sinkageDeformationModulusK * V.wheelWidth * sqrt(V.wheelsDiameter);
wheelSinkageZ = (numerator/denominator)^(2/((2 * V.N) + 1));


end

