%% Choppers
Chopper.eff = 0.95 ;    % Chopper efficiency

%% DC machine model (CIM Motor from VEX Robotics)
% https://www.systemvision.com/blog/first-robotics-frc-motor-modeling-may-6-2016

DCM.P_nom = 337 ;       % Nominal power [W]
DCM.W_nom = 5310 ;      % Nominal speed [rpm]
DCM.U_nom = 12 ;        % Nominal voltage [V]
DCM.I_nom = 133 ;       % Nominal current [A]
DCM.L = 5.90e-5 ;        % Armature inductance [H]
DCM.r = 0.0916 ;        % Armature resistance [Ohm]
DCM.phi = DCM.U_nom / ((DCM.W_nom * 2 * pi) / (60));
