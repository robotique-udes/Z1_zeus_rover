load cycle_ECE_mod ;
% load Cycle_OffRoad ;

%% Simulation configuration
SIMULPARAM.Ts = 1e-4;                       % Sampling time (2e-4 for Cycle_OffRoad)
SIMULPARAM.T_end = max(CYCL.time);          % Running time
SIMULPARAM.Decimation = 0.01/SIMULPARAM.Ts ;