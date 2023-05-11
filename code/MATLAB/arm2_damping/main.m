clear; close all; clc;


params.I = 10; % kg m^2
params.b = 0.1; % N m s
params.M = 1; % kg
params.L = 100; % m
params.g = 9.81; % m / s^2

y0 = [-pi/2, 0];
[t, y] = ode23(@(t,y) odefun(t, y, params), [0, 10], y0);

plot(t, y(:,1))