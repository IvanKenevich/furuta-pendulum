clear; close all; clc;

%% actual behavior
m = 0.01899165; % kg, mass of arm2
l_cg = 0.09155541; % m

% Itot = 0.00020118; % kg-m^2
Itot = 0.00027118; % kg-m^2

x = table2array(readtable("dataset4.txt"));
x = x(341:3467, :); % cut out the relevant portion
x(:,1) = 1e-6 * x(:,1); % microsecond to second conversion
x(:,1) = x(:,1) - x(1,1); % remove time offset
% figure
% plot(x(:,2))

%% theoretical solution
I0 = Itot;
b0 = 4e-5; % least squares comes up with 3.5072e-05, which is 13% away
M0 = m;
L0 = l_cg;
params.I = I0; % kg m^2
params.b = b0; % N m s
params.M = M0; % kg
params.L = L0; % m
params.g = 9.81; % m / s^2

% % uncomment for least squares solution
% lb = [I0 * 0.5, b0 * 0.1, M0 * 0.5, L0 * 0.5];
% ub = [I0 * 1.5, b0 * 10, M0 * 1.5, L0 * 1.5];
% sol = fmincon(@(p) cost_func(x, p(1), p(2), p(3), p(4), params), [I0, b0, M0, L0], [], [], [], [], lb, ub)
% 
% params.I = sol(1);
% params.b = sol(2);
% params.M = sol(3);
% params.L = sol(4);
% y0 = [x(1,2) * pi/180, 0];
% [t, y] = ode23(@(t,y) odefun(t, y, params), x(:,1), y0);

figure
plot(t, y(:,1)*180/pi, x(:,1), x(:,2))
legend(["Simulation", "Experiment"])

%% optimization function
function sse = cost_func(x, I, b, M, L, params)
    params.I = I;
    params.b = b;
    params.M = M;
    params.L = L;
    y0 = [x(1,2) * pi/180, 0];
    [t, y] = ode23(@(t,y) odefun(t, y, params), x(:,1), y0);
    sse = sum((y(:,1) - x(:,2)).^2);
end