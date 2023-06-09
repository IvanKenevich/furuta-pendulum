clear; close all; clc;
load("../paper/derived_systems.mat")

%% load and process actual behavior
x = readmatrix("dataset1.txt");
x = x(474:883, :); % cut out the relevant portion
x(:,1) = 1e-6 * x(:,1); % microsecond to second conversion
x(:,1) = x(:,1) - x(1,1); % remove time offset

t = x(:,1);
V = x(:,2);
t1 = x(:,3);
t2 = x(:,4);

% figure
% hold on
% plot(t, V / 4 * 100 - 50)
% plot(t, mod(t1, 360))
% plot(t, t2)
% hold off

%% simulate
tspan = [t(1), t(end)]; %t;
y0 = [t1(1) t2(1) 0 0 0];
Vfun = @(tq) interp1(t, V, tq);
% solver_options = odeset('RelTol',1e-6, 'AbsTol',1e-10);
[t_sim, y_sim] = ode45(@(t, y) sys_odefun(t, y, f_ddt1, f_ddt2, Vfun(t), motor_pars), tspan, y0);

t1_sim = rad2deg(y_sim(:,1));
t2_sim = rad2deg(y_sim(:,2));
i_sim = y_sim(:,5);

ax1 = subplot(3,1,1);
plot(t, t1, t_sim, t1_sim)
legend(["real", "simulated"]);
ylabel("Arm 1 [deg]")

ax2 = subplot(3,1,2);
plot(t, t2, t_sim, t2_sim)
ylabel("Arm 2 [deg]")

ax3 = subplot(3,1,3);
hold on
plot(t, V)
% plot(t_sim, i_sim)
hold off
ylabel("Input voltage [V]")
xlabel("Time [s]")
linkaxes([ax1, ax2, ax3], 'x')