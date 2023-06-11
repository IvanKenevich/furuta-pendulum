clear; clc; close all;
load("../paper/derived_systems.mat")

sys = systems.full_measured;

%% design gain matrix
tr = 0.5;
PMO = 10;

z=((-log(PMO/100))/(sqrt(log(PMO/100)^(2)+pi^(2))));
wn=((2.917*(z^(2)-0.142852245458*z+0.342817963661))/(tr));

p1=-z*wn+wn*sqrt(1-z^2)*1i;
p2=-z*wn-wn*sqrt(1-z^2)*1i;
p3 = floor(5*real(p1));
p4 = 1.1*p3;
p5 = 1.1*p4;

p = [p1 p2 p3 p4 p5];
K = place(sys.A, sys.B, p);

sys_cl = sys;
sys_cl.A = sys.A - sys.B * K;

%% design observer
addpath("../utils")
[sys_obs, L] = Observer(sys, K, p(5), 4);

t = linspace(0, 4, 1000);
sys_cl = ss(sys_cl.A, sys_cl.B, eye(5), zeros(5,1));
sys_obs = ss(sys_obs.A, sys_obs.B, [zeros(5,5), eye(5)], zeros(5,1));
[y_c, ~] = initial(sys_cl, [1 1 1 1 1], t);
[y_o, ~] = initial(sys_obs,[[1 1 1 1 1],[0 0 0 0 0]], t);
figure
hold on
plot(t,y_c)
set(gca,'ColorOrderIndex',1) % reset color indexing so that colors match for y_c and y_o
plot(t,y_o,'--')
title('Free Response with and without Observer')
subtitle('Different ICs')
ylabel('Amplitude')
xlabel('Time (s)')
hold off

%% simulate nonlinear system with a linear controller
tspan = [0, 5];
y0 = [0 deg2rad(165) 0 0 0];
control_pars.K = K;
[t, y] = ode45(@(t, y) sys_odefun(t, y, f_ddt1, f_ddt2, control_law(t, y, control_pars), motor_pars), tspan, y0);
% torque = (y - ref) * -K';

for row=1:height(y)
    yrow = y(row, :);
    
    V(row) = - K * (yrow - [yrow(1) pi 0 0 0])';
    if abs(V(row)) > motor_pars.Vmax
        V(row)= sign(V(row)) * 12;
    end
end

figure
subplot(2,1,1)
plot(t, y(:,1)*180/pi, t, y(:,2)*180/pi), title("Arm positions"), xlabel("Time [s]"), ylabel("Anle [deg]"), ylim([180 - 360, 180 + 360]), grid on
subplot(2,1,2)
plot(t, y(:,3)*180/pi/360, t, y(:,4)*180/pi/360), title("Arm velocities"), xlabel("Time [s]"), ylabel("Velocity [rps]"), grid on


function [out] = control_law(t, y, pars)
    out = - pars.K * (y - [y(1) pi 0 0 0]');
end