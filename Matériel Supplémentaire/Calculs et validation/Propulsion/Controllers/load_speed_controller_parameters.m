%% Control
C_speed.tau = 70;
C_speed.rise_time = 1;
C_speed.zeta = 1;
C_speed.wn = 4.744 / C_speed.rise_time;
C_speed.kp = 2 * C_speed.zeta * C_speed.wn * C_speed.tau;
C_speed.ki = (C_speed.wn ^ 2) * C_speed.tau;