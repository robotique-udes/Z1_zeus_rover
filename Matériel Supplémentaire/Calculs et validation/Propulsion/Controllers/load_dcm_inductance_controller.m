C_current.tau = DCM.L / DCM.r;
C_current.rise_time = 1;
C_current.zeta = 1;
C_current.wn = 4.744 / C_current.rise_time;
C_current.kp = 2 * C_current.zeta * C_current.wn * C_current.tau;
C_current.ki = (C_current.wn ^ 2) * C_current.tau;