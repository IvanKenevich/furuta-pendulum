clear; clc; close all;
load("../paper/derived_systems.mat")

sys = systems.full_measured;

%% design gain matrix
Ts = 1e-3;
tr = 0.3;
PMO = 10;

z=((-log(PMO/100))/(sqrt(log(PMO/100)^(2)+pi^(2))));
wn=((2.917*(z^(2)-0.142852245458*z+0.342817963661))/(tr));

p1=-z*wn+wn*sqrt(1-z^2)*1i;
p2=-z*wn-wn*sqrt(1-z^2)*1i;
p3 = floor(20*real(p1));
p4 = 1.1*p3;
p5 = 1.1*p4;

p = [p1 p2 p3 p4 p5];
K = place(sys.A, sys.B, p);
K(1) = 0;

sys_cl = sys;
sys_cl.A = sys.A - sys.B * K;

% digital controller design
sysD = c2d(sys,Ts);
pd = [exp(p(1)*Ts) exp(p(2)*Ts) exp(p(3)*Ts) exp(p(4)*Ts) exp(p(5)*Ts)];
Kd = place(sysD.A,sysD.B,pd);
Kd(1) = 0;

sys_cl_D = sysD;
sys_cl_D.A = sysD.A - sysD.B * Kd;

%% design observer
addpath("../utils")
[sys_obs, L] = Observer(sysD, Kd, p(1), 4, true, Ts);

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

%% print C code
A_1 = sysD.A - sysD.B * Kd - L * sysD.C; % A_1 from Turcic observer handout
BD = sysD.B;


fprintf('#define TIMER_CYCLE_MICROS %g\n',Ts*1e6)

fprintf('Matrix<1,5> Kd = {%g, %g, %g, %g, %g};\n', Kd(1), Kd(2), Kd(3), Kd(4), Kd(5));
fprintf(['Matrix<5,5> A_1 = {%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g,\n' ...
                            '%g, %g, %g, %g, %g};\n'], ...
                            A_1(1, 1), A_1(1,2), A_1(1,3), A_1(1,4), A_1(1,5), ...
                            A_1(2, 1), A_1(2,2), A_1(2,3), A_1(2,4), A_1(2,5), ...
                            A_1(3, 1), A_1(3,2), A_1(3,3), A_1(3,4), A_1(3,5), ...
                            A_1(4, 1), A_1(4,2), A_1(4,3), A_1(4,4), A_1(4,5), ...
                            A_1(5, 1), A_1(5,2), A_1(5,3), A_1(5,4), A_1(5,5) ...
)
fprintf('Matrix<5,1, Array<5,1,volatile float> > B = {%g, %g, %g, %g, %g};\n', BD(1), BD(2), BD(3), BD(4), BD(5))
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

%% functions
function [out] = control_law(t, y, pars)
    out = - pars.K * (y - [y(1) pi 0 0 0]');
%     out = - pars.K * y;
end