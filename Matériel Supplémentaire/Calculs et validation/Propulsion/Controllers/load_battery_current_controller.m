%% Battery current controller
L.BAT.L = 0.2e-3 ;  % Battery inductor inductance [H]
L.BAT.r = 0.01 ;    % Battery inductor resistance [Ohm]
L.BAT.K = 1 / L.BAT.r ;
L.BAT.Tau = L.BAT.L / L.BAT.r ;
BATCURRENTCONTROL.Tres = 10e-3 ;  % Response time
% Controller selection
% 1: PI control
% 2: IP control
Bat_current_control = 2 ;
% PI controller
BATCURRENTCONTROL.Ki_pi = 5 /(BATCURRENTCONTROL.Tres*L.BAT.K); % 5 times of system time constant
BATCURRENTCONTROL.Kp_pi = L.BAT.Tau*BATCURRENTCONTROL.Ki_pi ;
% IP controller
BATCURRENTCONTROL.xi = 1 ;
BATCURRENTCONTROL.wn = 2*pi/BATCURRENTCONTROL.Tres ;
BATCURRENTCONTROL.Kp_ip = (2*BATCURRENTCONTROL.xi*BATCURRENTCONTROL.wn*L.BAT.Tau - 1)/L.BAT.K ;
BATCURRENTCONTROL.Ki_ip = BATCURRENTCONTROL.wn^2*L.BAT.Tau/L.BAT.K ;

%% H-ESS EMS
% Filtering strategy
STRATEGY.f_c = 0.02 ; % LPF cut-off frequency [Hz]