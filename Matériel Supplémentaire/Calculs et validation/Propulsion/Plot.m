close all ;
clc
%% Global setting
font = 'Times New Roman';% Fontname ('Times New Roman' for article, 'Arial' for presentation)
fontsize = 14 ;% Fontsize (8-9 for article, 16 for presentation)
fontsize_legend = 14; % Legend Fontsize (8-9 for article, 16 for presentation)

v_veh       = ScopeData.signals(1).values(:,1) ;
v_veh_ref  	= ScopeData.signals(1).values(:,2) ;

T_em        = ScopeData.signals(11).values(:,1) ;

u_ac        = ScopeData.signals(2).values(:,1) ;
u_bc        = ScopeData.signals(2).values(:,2) ;
u_dc        = ScopeData.signals(2).values(:,3) ;
u_dc_ref   	= ScopeData.signals(2).values(:,4) ;

phi         = ScopeData.signals(6).values(:,1) ;
phi_ref    	= ScopeData.signals(6).values(:,3) ;

i_d         = ScopeData.signals(5).values(:,1) ;
i_d_ref    	= ScopeData.signals(5).values(:,2) ;

i_q         = ScopeData.signals(4).values(:,1) ;
i_q_ref    	= ScopeData.signals(4).values(:,2) ;

u_sc        = ScopeData.signals(8).values(:,1) ;
u_bat       = ScopeData.signals(8).values(:,2) ;
u_sc_max    = ScopeData.signals(8).values(:,3) ;
u_sc_min    = ScopeData.signals(8).values(:,4) ;

i_tot   	= ScopeData.signals(9).values(:,1) ;
i_ch_bat   	= ScopeData.signals(9).values(:,2) ;
i_ch_sc   	= ScopeData.signals(9).values(:,3) ;

%% Vehicle velocity 
figure('units','centimeters','position',[5 2 [16 6]]);% Creation of the figure area  
hold on
h1 = plot(time, v_veh(1:end), 'b-', 'LineWidth',2) ;
h2 = plot(time, v_veh_ref(1:end), 'k--', 'LineWidth',1) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[km/h]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [0 45]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
ylim(limites_y)% limits for y axis display
title('Vehicle velocity [km/h]')
legend([h1, h2], {'\it{v_{\rm{veh}}}', '\it{v_{\rm{veh ref}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

%% Electrical machine torque
figure('units','centimeters','position',[5 2 [16 6]]);% Creation of the figure area  
hold on
h1 = plot(time, T_em(1:end), 'b-', 'LineWidth',2) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[Nm]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [-10 20]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
% ylim(limites_y)% limits for y axis display
title('Electrical machine torque [Nm]')
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

%% Drive subsystem voltages
figure('units','centimeters','position',[5 2 [16 8]]);% Creation of the figure area  
hold on
h1 = plot(time, u_ac(1:end), 'r--', 'LineWidth',1) ;
h2 = plot(time, u_bc(1:end), 'k-', 'LineWidth',1) ;
h3 = plot(time, u_dc(1:end), 'r-', 'LineWidth',2) ;
h4 = plot(time, u_dc_ref(1:end), 'k--', 'LineWidth',1) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[V]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [-65 65]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
% ylim(limites_y)% limits for y axis display
title('Drive subsystem voltages [V]')
legend([h1, h2, h3, h4], {'\it{u_{\rm{ac}}}', '\it{u_{\rm{bc}}}', '\it{u_{\rm{dc}}}', '\it{u_{\rm{dc ref}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

% figure('units','centimeters','position',[5 2 [10 6]]);% Creation of the figure area  
% hold on
% h1 = plot(time, u_ac(1:end), 'r--', 'LineWidth',1) ;
% h2 = plot(time, u_bc(1:end), 'k-', 'LineWidth',1) ;
% h3 = plot(time, u_dc(1:end), 'r-', 'LineWidth',2) ;
% h4 = plot(time, u_dc_ref(1:end), 'k--', 'LineWidth',1) ;
% hold off
% axe_x= 'Time [s]';% Legend of x axis
% axe_y= '[V]';% Legend of y axis
% limites_x= [15.36 15.4]; % limits of x axis display
% limites_y= [-65 70]; % Limits of y axis display
% xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% % ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
% set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
% xlim(limites_x)% Limits for x axis display  
% ylim(limites_y)% limits for y axis display
% % title('Drive subsystem voltages [V]')
% % legend([h1, h2, h3, h4], {'\it{u_{\rm{ac}}}', '\it{u_{\rm{bc}}}', '\it{u_{\rm{dc}}}', '\it{u_{\rm{dc ref}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
% grid on;
% ax = gca ;
% ax.GridLineStyle = ':' ;
% ax.GridColor = 'k' ;
% ax.GridAlpha = 1 ;
% box on ;

%% Machine flux
figure('units','centimeters','position',[5 2 [16 6]]);% Creation of the figure area  
hold on
h1 = plot(time, phi(1:end), 'b-', 'LineWidth',2) ;
h2 = plot(time, phi_ref(1:end), 'k--', 'LineWidth',1) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[Wb]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [0 0.1]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
% ylim(limites_y)% limits for y axis display
title('Machine flux [Wb]')
legend([h1, h2], {'\it{\Phi_{\rm{}}}', '\it{\Phi_{\rm{ref}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

%% d-q currents
figure('units','centimeters','position',[5 2 [16 6]]);% Creation of the figure area  
hold on
h1 = plot(time, i_d(1:end), 'r-', 'LineWidth',2) ;
h2 = plot(time, i_d_ref(1:end), 'k--', 'LineWidth',1) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[A]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [0 300]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
% ylim(limites_y)% limits for y axis display
title('d-axis current [A]')
legend([h1, h2], {'\it{i_{\rm{d}}}', '\it{i_{\rm{d ref}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

figure('units','centimeters','position',[5 2 [16 6]]);% Creation of the figure area  
hold on
h1 = plot(time, i_q(1:end), 'r-', 'LineWidth',2) ;
h2 = plot(time, i_q_ref(1:end), 'k--', 'LineWidth',1) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[A]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [-150 250]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
ylim(limites_y)% limits for y axis display
title('q-axis current [A]')
legend([h1, h2], {'\it{i_{\rm{q}}}', '\it{i_{\rm{q ref}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

%% H-ESS voltages
figure('units','centimeters','position',[5 2 [16 6]]);% Creation of the figure area  
hold on
h1 = plot(time, u_bat(1:end), 'b-.', 'LineWidth',2) ;
h2 = plot(time, u_sc(1:end), 'r-', 'LineWidth',2) ;
h3 = plot(time, u_sc_max(1:end), 'k--', 'LineWidth',1) ;
h4 = plot(time, u_sc_min(1:end), 'k--', 'LineWidth',1) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[V]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [24 50]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
% ylim(limites_y)% limits for y axis display
title('H-ESS voltages [V]')
legend([h1, h2], {'\it{u_{\rm{bat}}}', '\it{u_{\rm{SC}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];

%% H-ESS currents
figure('units','centimeters','position',[5 2 [16 8]]);% Creation of the figure area  
hold on
h1 = plot(time, i_tot(1:end), 'k-.', 'LineWidth',1) ;
h2 = plot(time, i_ch_bat(1:end), 'b--', 'LineWidth',2) ;
h3 = plot(time, i_ch_sc(1:end), 'r-', 'LineWidth',2) ;
hold off
axe_x= 'Time [s]';% Legend of x axis
axe_y= '[V]';% Legend of y axis
limites_x= [0 time(end)]; % limits of x axis display
limites_y= [-150 160]; % Limits of y axis display
xlabel(axe_x,'Fontname',font,'Fontsize',fontsize)% Labelling x axis
% ylabel(axe_y,'Fontname',font,'Fontsize',fontsize)% Labelling y axis
set(gca,'FontSize',fontsize,'FontName',font)% sizing legend font and font size
xlim(limites_x)% Limits for x axis display  
ylim(limites_y)% limits for y axis display
title('H-ESS currents [A]')
legend([h1, h2, h3], {'\it{i_{\rm{tot}}}', '\it{i_{\rm{ch bat}}}', '\it{i_{\rm{ch SC}}}'}, 'Orientation', 'horizon', 'Location', 'Best', 'FontSize',fontsize_legend);
grid on;
ax = gca ;
ax.GridLineStyle = ':' ;
ax.GridColor = 'k' ;
ax.GridAlpha = 1 ;
box on ;

% To remove the outer space
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - 1.1*ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left*1.0 bottom*1.1 ax_width ax_height];