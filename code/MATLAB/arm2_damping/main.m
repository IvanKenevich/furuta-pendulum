clear; close all; clc;

%% actual behavior
m = 30.38172e-3; % kg, mass of arm2
l_cg = 0.05447391; % m

Itot = 0.19735381e-3; % kg-m^2

x = table2array(readtable("dataset3.txt"));
x = x(432:795, :); % cut out the relevant portion
x(:,1) = 1e-6 * x(:,1); % microsecond to second conversion
x(:,1) = x(:,1) - x(1,1); % remove time offset
% figure
% plot(x(:,1), x(:,2))

%% theoretical solution
params.I = Itot; % kg m^2
params.b = 9e-5; % N m s
params.M = m; % kg
params.L = l_cg; % m
params.g = 9.81; % m / s^2

y0 = [x(1,2) * pi/180, 0];
[t, y] = ode23(@(t,y) odefun(t, y, params), [0, 7], y0);

figure
plot(t, y(:,1)*180/pi, x(:,1), x(:,2))