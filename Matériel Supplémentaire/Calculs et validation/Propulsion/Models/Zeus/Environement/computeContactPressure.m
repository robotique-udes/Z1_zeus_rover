function [contactPressure] = computeContactPressure(C, V, wheelSinkageZ)

contactPressure = ((C.cohesiveModulusKc / V.wheelWidth) + C.frictionalModulusKphi)*(wheelSinkageZ^V.n);

end

