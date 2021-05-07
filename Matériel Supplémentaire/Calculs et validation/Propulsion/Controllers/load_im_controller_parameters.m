%% DC bus voltage controller
DCBUSCONTROL.u_dc_ref = 60 ;        % DC bus voltage reference [V]
DCBUSCONTROL.K_cap = 1/DCCAP.C ;    % DC bus capacitor gain
DCBUSCONTROL.T_res = 10*max(BATCURRENTCONTROL.Tres, SCCURRENTCONTROL.Tres) ;      % Response time [s]
DCBUSCONTROL.xi = 1 ; % Damping factor; 1 means without overshoot
DCBUSCONTROL.wn = 2*pi/DCBUSCONTROL.T_res ; % [rad/s] Natural frequency
% Controller selection
% 1: PI control
% 2: IP control
DC_bus_voltage_control = 1 ;
% PI controller 
DCBUSCONTROL.Kp_pi = 2*DCBUSCONTROL.xi*DCBUSCONTROL.wn*DCCAP.C ;
DCBUSCONTROL.Ki_pi = DCBUSCONTROL.wn^2*DCCAP.C ;
% IP controller
DCBUSCONTROL.Kp_ip = 2*DCBUSCONTROL.xi*DCBUSCONTROL.wn*DCCAP.C ;
DCBUSCONTROL.Ki_ip = DCBUSCONTROL.wn^2*DCCAP.C ;

%% Motor d-current controllers
dCURRENTCONTROL.Tres = 2e-3;  % Response time [s]
% Controller selection
% 1: PI control
% 2: IP control
d_current_control = 2 ;
% PI controller
dCURRENTCONTROL.Ki_pi = 5/(dCURRENTCONTROL.Tres*IM.k_s);
dCURRENTCONTROL.Kp_pi = IM.tau_s*dCURRENTCONTROL.Ki_pi ;
% IP controller
dCURRENTCONTROL.xi = 1 ; % Damping factor; 1 means without overshoot
dCURRENTCONTROL.wn = 2*pi/dCURRENTCONTROL.Tres ;
dCURRENTCONTROL.Kp_ip = (2*dCURRENTCONTROL.xi*dCURRENTCONTROL.wn*IM.tau_s - 1)/IM.k_s ;
dCURRENTCONTROL.Ki_ip = dCURRENTCONTROL.wn^2*IM.tau_s/IM.k_s ;

%% Motor q-current controllers
qCURRENTCONTROL.Tres = 2e-3;  % Response time [s]
% Controller selection
% 1: PI control
% 2: IP control
q_current_control = 2 ;
% PI controller
qCURRENTCONTROL.Ki_pi = 5/(qCURRENTCONTROL.Tres*IM.k_s);
qCURRENTCONTROL.Kp_pi = IM.tau_s*qCURRENTCONTROL.Ki_pi ;
% IP controller
qCURRENTCONTROL.xi = 1 ; % Damping factor; 1 means without overshoot
qCURRENTCONTROL.wn = 2*pi/qCURRENTCONTROL.Tres ;
qCURRENTCONTROL.Kp_ip = (2*qCURRENTCONTROL.xi*qCURRENTCONTROL.wn*IM.tau_s - 1)/IM.k_s ;
qCURRENTCONTROL.Ki_ip = qCURRENTCONTROL.wn^2*IM.tau_s/IM.k_s ;

%% Motor flux controller
FLUXCONTROL.Tres = 10*dCURRENTCONTROL.Tres ; % Response time [s]
FLUXCONTROL.xi = 1 ; % Damping factor; 1 means without overshoot
FLUXCONTROL.wn = 2*pi/FLUXCONTROL.Tres ;
% Controller selection
% 1: PI control
% 2: IP control
Flux_control = 1 ;
% PI controller
FLUXCONTROL.Ki_pi = 5/(FLUXCONTROL.Tres*IM.Lm) ;
FLUXCONTROL.Kp_pi = IM.tau_r*FLUXCONTROL.Ki_pi ;
% IP controller
FLUXCONTROL.Kp_ip = (2*FLUXCONTROL.xi*FLUXCONTROL.wn*IM.tau_r - 1)/IM.Lm ;
FLUXCONTROL.Ki_ip = FLUXCONTROL.wn^2*IM.tau_r/IM.Lm ;

%% Vehicle speed controller
SPEEDCONTROL.Tres = 2 ;     % [s] Response time
SPEEDCONTROL.xi = 1 ;      % Damping factor; 1 means without overshoot
SPEEDCONTROL.wn = 2*pi/SPEEDCONTROL.Tres ; % [rad/s] Natural frequency
% Controller selection
% 1: PI control
% 2: IP control
Speed_control = 2 ;
% PI controller
SPEEDCONTROL.Kp_pi = 2*SPEEDCONTROL.xi*SPEEDCONTROL.wn*CHAS.M_veh_tot ;
SPEEDCONTROL.Ki_pi = SPEEDCONTROL.wn^2*CHAS.M_veh_tot;
% IP controller
SPEEDCONTROL.Kp_ip = 2*SPEEDCONTROL.xi*SPEEDCONTROL.wn*CHAS.M_veh_tot ;
SPEEDCONTROL.Ki_ip = SPEEDCONTROL.wn^2*CHAS.M_veh_tot;

%% Field weakening strategy
FLUXSTRATEGY.Phi_ref = (DCBUSCONTROL.u_dc_ref/sqrt(3) - 400*IM.Rs)/(2*pi*IM.f_nom) ;