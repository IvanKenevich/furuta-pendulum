clear; clc; close all;
load("../paper/derived_systems.mat")

sys = systems.full_measured;

%% design gain matrix
Ts = 1e-3;
tr = 0.5;
PMO = 25;

z=((-log(PMO/100))/(sqrt(log(PMO/100)^(2)+pi^(2))));
wn=((2.917*(z^(2)-0.142852245458*z+0.342817963661))/(tr));

p1=-z*wn+wn*sqrt(1-z^2)*1i;
p2=-z*wn-wn*sqrt(1-z^2)*1i;
p3 = floor(20*real(p1));
p4 = 1.1*p3;
p5 = 1.1*p4;

p = [p1 p2 p3 p4 p5];
K = place(sys.A, sys.B, p);
% K(1) = 0;

sys_cl = sys;
sys_cl.A = sys.A - sys.B * K;

% digital controller design
sysD = c2d(sys,Ts);
pd = [exp(p(1)*Ts) exp(p(2)*Ts) exp(p(3)*Ts) exp(p(4)*Ts) exp(p(5)*Ts)];
Kd = place(sysD.A,sysD.B,pd);
% Kd(1) = 0;

sys_cl_D = sysD;
sys_cl_D.A = sysD.A - sysD.B * Kd;

%% design observer
addpath("../utils")
[sys_obs, L] = Observer(sysD, Kd, p(1), 50, true, Ts);

t = 0:Ts:4;
% sys_obs = ss(sys_obs.A, sys_obs.B, [zeros(5,5), eye(5)], zeros(5,1));
[y_c, ~, x_c] = initial(sys_cl_D, [1 1 1 1 1], t);
[y_o, ~, x_o] = initial(sys_obs,[[1 1 1 1 1],[0 0 0 0 0]], t);
figure
hold on
plot(t,x_c)
set(gca,'ColorOrderIndex',1) % reset color indexing so that colors match for y_c and y_o
plot(t,x_o(:,(height(sys_cl_D.A) + 1):end),'--')
title('Free Response with and without Observer')
subtitle('Different ICs')
ylabel('Amplitude')
xlabel('Time (s)')
hold off

%% simulate nonlinear system with a linear controller
tvals = linspace(0,5,1e4);
y0 = [0 deg2rad(180 + 15) 0 0 0];
control_pars.K = K;
[t, y] = ode45(@(t, y) sys_odefun(t, y, f_ddt1, f_ddt2, control_law(t, y, control_pars), motor_pars), tvals, y0);

% torque = (y - ref) * -K';

% saturation actually implemented inside sys_odefun
% this exists purely to adjust input voltage post-factum in case you want
% to plot it
for row=1:height(y)
    yrow = y(row, :);
    
    V(row) = - K * (yrow - [yrow(1) pi 0 0 0])';
    if abs(V(row)) > motor_pars.Vmax
        V(row)= sign(V(row)) * 12;
    end
end

figure
ax1 = subplot(2,1,1);
plot(t, y(:,1)*180/pi, t, y(:,2)*180/pi), title("Arm positions"), xlabel("Time [s]"), ylabel("Angle [deg]"), ylim([180 - 360, 180 + 360]), grid on
ax2 = subplot(2,1,2);
plot(t, y(:,3)*180/pi/360, t, y(:,4)*180/pi/360), title("Arm velocities"), xlabel("Time [s]"), ylabel("Velocity [rps]"), grid on
linkaxes([ax1, ax2], 'x')

%% simulate linear system with a linear controller (and compare to nonlinear system)
y0_lin = y0 - deg2rad([0 180 0 0 0]);
[~, y_lin] = ode45(@(t, y) sys_odefun_lin(t, y, sys, control_law_lin(t, y, control_pars), motor_pars), tvals, y0_lin);
y_lin = y_lin + deg2rad([0 180 0 0 0]);

figure
ax1 = subplot(2,1,1);
hold on
plot(t, y(:,1)*180/pi, 'm', t, y(:,2)*180/pi, 'c'), title("Arm positions"), xlabel("Time [s]"), ylabel("Angle [deg]"), ylim([180 - 360, 180 + 360]), grid on
plot(t, y_lin(:,1)*180/pi, 'k--', t, y_lin(:,2)*180/pi, 'b--')
legend(["Arm 1", "Arm 2", "Arm 1 linearized", "Arm 2 linearized"], location="best")
hold off
ax2 = subplot(2,1,2);
hold on
plot(t, y(:,3)*180/pi/360, 'm', t, y(:,4)*180/pi/360, 'c'), title("Arm velocities"), xlabel("Time [s]"), ylabel("Velocity [rps]"), grid on
plot(t, y(:,3)*180/pi/360, 'k--', t, y(:,4)*180/pi/360, 'b--')
legend(["Arm 1", "Arm 2", "Arm 1 linearized", "Arm 2 linearized"], location="best")
hold off
linkaxes([ax1, ax2], 'x')


%% print C code
A_1 = sysD.A - sysD.B * Kd - L * sysD.C; % A_1 from Turcic observer handout
A = sysD.A;
BD = sysD.B;
C = sysD.C;


fprintf('#define TIMER_CYCLE_MICROS %g\n \n\n\n\n\n',Ts*1e6)

fprintf('Matrix<1,5> Kd = {%g, %g, %g, %g, %g};\n', Kd(1), Kd(2), Kd(3), Kd(4), Kd(5));
fprintf(['Matrix<5,5> A = {%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g};\n'], ...
                            A(1, 1), A(1,2), A(1,3), A(1,4), A(1,5), ...
                            A(2, 1), A(2,2), A(2,3), A(2,4), A(2,5), ...
                            A(3, 1), A(3,2), A(3,3), A(3,4), A(3,5), ...
                            A(4, 1), A(4,2), A(4,3), A(4,4), A(4,5), ...
                            A(5, 1), A(5,2), A(5,3), A(5,4), A(5,5) ...
)
fprintf('Matrix<5,1, Array<5,1,volatile float> > B = {%g, %g, %g, %g, %g};\n', BD(1), BD(2), BD(3), BD(4), BD(5))
fprintf(['Matrix<4,5> C = {%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g};\n'], ...
                            C(1, 1), C(1,2), C(1,3), C(1,4), C(1,5), ...
                            C(2, 1), C(2,2), C(2,3), C(2,4), C(2,5), ...
                            C(3, 1), C(3,2), C(3,3), C(3,4), C(3,5), ...
                            C(4, 1), C(4,2), C(4,3), C(4,4), C(4,5) ...
)
fprintf(['Matrix<5,4> L = {%g, %g, %g, %g,\n' ...
                           '%g, %g, %g, %g,\n' ...
                           '%g, %g, %g, %g,\n' ...
                           '%g, %g, %g, %g,\n' ...
                           '%g, %g, %g, %g};\n'], ...
                           L(1, 1), L(1,2), L(1,3), L(1,4), ...
                           L(2, 1), L(2,2), L(2,3), L(2,4), ...
                           L(3, 1), L(3,2), L(3,3), L(3,4), ...
                           L(4, 1), L(4,2), L(4,3), L(4,4), ...
                           L(5, 1), L(5,2), L(5,3), L(5,4) ...
)


save('controller_parameters.mat', 'sys', 'K', 'sys_cl', 'Ts', 'sysD', 'Kd', 'sys_cl_D', 'L', 'sys_obs')

%% classical controller design
s = tf('s');
tfs = tf(sys);
arm2tf = tfs(2);
% arm2tf.num = round(arm2tf.num{1}, 4);

polezero = zpk(arm2tf);
cancel_term = (s - polezero.P{1}(1)); % REMEMBER ME -- Add to MCU code
polezero.P = polezero.P{1}(2:end);

controller = pidtune(polezero, 'PIDF');
sys_cl = feedback(polezero * controller, 1);
sys_Vin = feedback(controller, polezero);

% [y2 t] = impulse(sys_cl, 1);
% vin = impulse(sys_Vin,t);
% y1 = lsim(tfs(1) * cancel_term,vin,t);
% y3 = lsim(tfs(3) * cancel_term,vin,t);
% y4 = lsim(tfs(4) * cancel_term,vin,t);
% 
% figure
% ax1 = subplot(5,1,1);
% plot(t,y1)
% ax2 = subplot(5,1,2);
% plot(t,y2)
% ax3 = subplot(5,1,3);
% plot(t,y3)
% ax4 = subplot(5,1,4);
% plot(t,y4)
% ax5 = subplot(5,1,5);
% plot(t,vin)
% linkaxes([ax1, ax2, ax3, ax4, ax5], 'x')
% 
% figure
% t = linspace(0,0.6,1e3);
% u = deg2rad(15)/0.257*deg2rad(15)*ones(size(t));
% u(t>0.3) = 0;
% lsim(sys_cl,u,t)
% xlim([0.3, 0.6])
% 
% figure
% lsim(sys_Vin,u,t)
% xlim([0.3, 0.6])
% 
% figure
% y2 = rad2deg(lsim(sys_cl,u,t));
% plot(t(t>0.3)-0.3,y2(t>0.3))

true_controller = cancel_term * controller;

%% functions
function [out] = control_law(t, y, pars)
    out = - pars.K * (y - [y(1) pi 0 0 0]');
%     out = - pars.K * y;
end

function [out] = control_law_lin(t, y, pars)
    out = - pars.K * (y - [y(1) 0 0 0 0]');
%     out = - pars.K * y;
end