clear 
clc
addpath("Controllers")
addpath("Models")
addpath("Models/App 1")
addpath("Models/Zeus")
addpath("Models/Zeus/Environement")
addpath("Models/Zeus/Other")
addpath("Parameters")
addpath("Simulinks")

%% DCM
% run('./Parameters/load_simulation_parameters')
% 
% % Load According Models
% run('./Models/load_battery_model')
% run('./Models/load_dcm_model')
% run('./Models/load_gearbox_model')
% run('./Models/load_environment_model')
% 
% % Controllers
% run('./Controllers/load_battery_current_controller')
% run('./Controllers/load_dcm_inductance_controller')
% run('./Controllers/load_speed_controller_parameters')
%% FULL IM WITH BRAKE & ENERGY
% run('./Parameters/load_simulation_parameters')
% 
% % Load According Models
% run('./Models/load_battery_model')
% run('./Models/load_im_model')
% run('./Models/load_supercap_model')
% run('./Models/load_gearbox_model')
% run('./Models/load_environment_model')
% 
% % Controllers
% run('./Controllers/load_battery_current_controller')
% run('./Controllers/load_im_controller_parameters')
% run('./Controllers/load_speed_controller_parameters')

%% IM MAP
% run('./Parameters/load_simulation_parameters')
% 
% % Load According Models
% run('./Models/load_battery_model')
% run('./Models/load_im_map_model')
% run('./Models/load_gearbox_model')
% run('./Models/load_environment_model')
% 
% % Controllers
% run('./Controllers/load_speed_controller_parameters')

%% DCM ZEUS
generateInclinationCycle
load_simulation_parameters

% Load According Models
load_battery_model_zeus
load_dcm_model_zeus
load_gearbox_model_zeus

% Controllers
load_battery_current_controller
load_dcm_inductance_controller
load_speed_controller_parameters
