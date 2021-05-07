%% Batteries model
% EV31A-A lead acid battery
BAT.Q_bat = 72;                     % Cell storage capacity [Ah]
BAT.Ccapacity = BAT.Q_bat*3600 ;	% Cell equivalent capacity [F]
BAT.v_cell_nom = 12 ;               % Cell nominal voltage
BAT.v_cell_min = 11.4 ;             % Cell minimal voltage at 80% depth-of-discharge DoD
BAT.r = 3.4e-3 ;                    % Cell equivalent serial resistance [Ohm]
BAT.n_se = 1 ;                      % Number of cells in series
BAT.n_pa = 1 ;                     	% Number of branches in parallel
BAT.SoC_init = 100 ; 
BAT.cranking_amp = 975;
BAT.I_charge_max = 0.25*BAT.cranking_amp; %TODO
BAT.I_max = 352;%TODO


