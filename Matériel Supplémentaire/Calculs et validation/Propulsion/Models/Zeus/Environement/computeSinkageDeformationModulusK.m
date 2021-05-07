function [sinkageDeformationModulusK] = computeSinkageDeformationModulusK(C, V)
sinkageDeformationModulusK = (C.cohesiveModulusKc / V.wheelWidth) + C.frictionalModulusKphi;
end

