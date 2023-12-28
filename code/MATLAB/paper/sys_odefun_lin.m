function [dydt] = sys_odefun_lin(t, y, sys, V, motor_pars)
%ODEFUN_TORQUE Summary of this function goes here
%   Detailed explanation goes here
Vmax = motor_pars.Vmax;

if abs(V) > Vmax
    V = sign(V) * Vmax;
end

dydt = sys.A * y + sys.B * V;
end