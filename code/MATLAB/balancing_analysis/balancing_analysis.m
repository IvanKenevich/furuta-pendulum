clear; close all; clc;
load("../paper/derived_systems.mat")

%% load and process actual behavior
% x = readmatrix("dataset1.txt"); x = x(200:250, :); % vanilla, slow print sampling
% x = readmatrix("dataset2.txt"); x = x(865:900, :); % vanilla, faster print sampling
% x = readmatrix("dataset3.txt"); x = x(:, :); % better ICs
x = readmatrix("dataset4.txt"); x = x(:, :); % better ICs, I=V/R

x(:,11) = x(:,11) - x(1,11); % remove time offset
x(:,11) = 1e-6 * x(:,11); % microsecond to second conversion

y = x(:,6:9);
t = x(:,11);
u = x(:,10);

x = x(:,1:5);
x(:,1:4) = rad2deg(x(:,1:4)); % convert radians to degrees
y = rad2deg(y); % convert radians to degrees

ax1 = subplot(5,1,1); 
hold on
plot(t, x(:,1)); plot(t, y(:,1)); ylabel("x1: Arm 1 [deg]");
legend(["Estimated", "Measured"]);
hold off
ax2 = subplot(5,1,2);
hold on
plot(t, x(:,2)); plot(t, y(:,2)); ylabel("x2: Arm 2 [deg]")
hold off
ax3 = subplot(5,1,3);
hold on
plot(t, x(:,3)); plot(t, y(:,3)); ylabel("x3: Arm 1 speed [deg/s]")
hold off
ax4 = subplot(5,1,4);
hold on
plot(t, x(:,4)); plot(t, y(:,4)); ylabel("x4: Arm 2 speed [deg/s]")
hold off
ax5 = subplot(5,1,5);
hold on
plot(t, x(:,5)); plot(t, u); ylabel("x5: Current [A]");
legend(["Current", "Voltage"]); 
hold off

linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')

arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes'))

%% simulate
% tspan = [t(1), t(end)]; %t;
% y0 = [t1(1) t2(1) 0 0 0];
% Vfun = @(tq) interp1(t, V, tq);
% % solver_options = odeset('RelTol',1e-6, 'AbsTol',1e-10);
% [t_sim, y_sim] = ode45(@(t, y) sys_odefun(t, y, f_ddt1, f_ddt2, Vfun(t), motor_pars), tspan, y0);
% 
% t1_sim = rad2deg(y_sim(:,1));
% t2_sim = rad2deg(y_sim(:,2));
% i_sim = y_sim(:,5);
% 
% 
% figure
% ax1 = subplot(3,1,1);
% plot(t, t1, t_sim, t1_sim)
% legend(["real", "simulated"]);
% ylabel("Arm 1 [deg]")
% 
% ax2 = subplot(3,1,2);
% plot(t, t2, t_sim, t2_sim)
% ylabel("Arm 2 [deg]")
% 
% ax3 = subplot(3,1,3);
% hold on
% plot(t, V)
% % plot(t_sim, i_sim)
% hold off
% ylabel("Input voltage [V]")
% xlabel("Time [s]")
% linkaxes([ax1, ax2, ax3], 'x')