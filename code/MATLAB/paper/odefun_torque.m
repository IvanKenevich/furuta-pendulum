function [dydt] = odefun_torque(t, y, f_ddt1, f_ddt2, tau1)
%ODEFUN_TORQUE Summary of this function goes here
%   Detailed explanation goes here
t1 = y(1);
t2 = y(2);
dt1 = y(3);
dt2 = y(4);
tau2 = 0;%0.2*rand() - 0.1; % no disturbance or rand()

ddt1 = f_ddt1(dt1, dt2, t2, tau1, tau2);
ddt2 = f_ddt2(dt1, dt2, t2, tau1, tau2);

dydt = [dt1; dt2; ddt1; ddt2];
end