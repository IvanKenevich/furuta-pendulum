clear; close all; clc;

%% actual behavior
l = 0.1459992; % m, length of arm 2
m = 5.31e-3; % kg, mass of arm2
m_clip = 8.32e-3; %kg, mass of clip
l_cg = ((m * l/2) + m_clip * l) / (m + m_clip);

Itot = (1/3)*m*l^2 + m_clip * l^2;

x = table2array(readtable("dataset2_clip.txt"));
x = x(272:1233, :); % cut out the relevant portion
x(:,1) = 1e-6 * x(:,1); % microsecond to second conversion
x(:,1) = x(:,1) - x(1,1); % remove time offset
figure
plot(x(:,1), x(:,2))

%% theoretical solution
params.I = Itot; % kg m^2
params.b = 3.5e-5; % N m s
params.M = m + m_clip; % kg
params.L = l_cg; % m
params.g = 9.81; % m / s^2

y0 = [-73.34 * pi/180, 0];
[t, y] = ode23(@(t,y) odefun(t, y, params), [0, 20], y0);

figure
plot(t, y(:,1)*180/pi, x(:,1), x(:,2))