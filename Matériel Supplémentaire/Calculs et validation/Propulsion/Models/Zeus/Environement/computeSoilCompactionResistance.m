function [soilCompactionResistance] = computeSoilCompactionResistance(C, V)
sinkageZ = computeSinkageZ(C, V);
sinkageDeformationModulusK = computeSinkageDeformationModulusK(C, V);
soilCompactionResistance = (V.wheelsNumber * V.wheelWidth * sinkageDeformationModulusK)/(V.N + 1) * sinkageZ^(V.N + 1);
end

