function [C,V] = loadConstants(inclination, nominalLinearSpeedNeeded)

inclinationInRadians = (inclination/360)*2*pi;

% Worst Constants for slope case
C.vehicleMass = 50;
C.g = 9.81;
C.accelerationNeeded = 1;

% Worst Constants for High-speed on flat
C.rhoAir = 1.1839; % At 25 degsC
C.vehiculeFrontArea = 0.5*0.5;
C.dragCoefficient = 1.05;

% Worst Constants for Buldozing Case 
wheelFlexibleDeformation = 0.20; % Percentage
C.wheelFlexibleDeformation = wheelFlexibleDeformation;
C.ny = 42;
C.nc = 58;
% C.soilFrictionAngle = 35;
C.soilFrictionAngleInRadians = (35/360)*2*pi;
C.soilVolumicMassY = 2500;
C.cohesiveModulusKc = 1500;
C.frictionalModulusKphi = 100;
C.soilInternalCohesionC = 170;

C.inclinationInRadians = inclinationInRadians;
C.nominalLinearSpeedNeeded = nominalLinearSpeedNeeded;

% V.wheelD = 0.15;
wheelR = (0.15 / 2);
V.wheelR = wheelR - (wheelR * wheelFlexibleDeformation);
wheelN = 6;
C.wheelN = wheelN;
V.wheelWidth = 0.08;
wheelsDiameter = wheelR * 2;
V.wheelsDiameter = wheelsDiameter;
V.wheelsNumber = wheelN;
V.N = 1; % Not the same N as n (number of wheel). Used for soil calculations  
V.motorsN = 6;
V.circonferenceInMeters = pi * wheelsDiameter;

end







