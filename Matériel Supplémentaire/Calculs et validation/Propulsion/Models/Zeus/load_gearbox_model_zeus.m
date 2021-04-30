%% Mechanical transmission
TRANS.k_gear = 40;                       % Gearbox ratio
TRANS.eff = 0.80;                       % Efficiency of the transmission
TRANS.R_wheel = 0.075;                 % Radius of the wheel [m]
TRANS.k = TRANS.k_gear/TRANS.R_wheel;  