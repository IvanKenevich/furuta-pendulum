clear; close all; clc;

% get the symbolically derived coefficients and functions
[up_lin_coeff, dn_lin_coeff, f1, f2, f3] = derive_coefficients(false);

% % substitute physical values from the paper
% L1 = 0.278; L2 = 0.3; l1 = 0.15; l2 = 0.148; % m
% m1 = 0.3; m2 = 0.075; % kg
% I1 = 2.48e-2; I2 = 3.86e-3; % kg-m^2
% b1 = 1e-4; b2 = 2.8e-4; % N-m-s
% g = 9.81; % m/s^2
% Lm = 0.005; % Henry
% Rm = 7.8; % Ohm
% Km = 0.09; % N-m/A

% substitute physical values from the design
L1 = 0.199; L2 = 0.144; l1 = 0.064888; l2 = 0.020327; % m
m1 = 48.022393e-3; m2 = 19.588257e-3; % kg
J1xx = 0.010807e-3; J1xy = -0.000018e-3; J1xz = -0.011989e-3; % kg-m^2
J1yy = 0.054190e-3; J1yz = 0;
J1zz = 0.048125e-3;
J2xx = 0.003293e-3; J2xy = 0; J2xz = -0.005240e-3;
J2yy = 0.034100e-3; J2yz = 0;
J2zz = 0.031078e-3;
b1 = 1e-4; b2 = 2.8e-4; % N-m-s
g = 9.81; % m/s^2
Lm = 0.005; % Henry
Rm = 7.8; % Ohm
Km = 0.09; % N-m/A

cf = up_lin_coeff; % select the linearization about the up position
cf = double(subs(cf));

[A31, A32, A33, A34, B31, B32, A41, A42, A43, A44, B41, B42] = deal(...
    cf(1,1), cf(1,2), cf(1,3), cf(1,4), cf(1,5), cf(1,6), ...
    cf(2,1), cf(2,2), cf(2,3), cf(2,4), cf(2,5), cf(2,6));

f_ddt1 = matlabFunction(vpa(subs(f1)));
f_ddt2 = matlabFunction(vpa(subs(f2)));
f_dtau1 = matlabFunction(vpa(subs(f3)));

clearvars -except A31 A32 A33 A34 B31 B32 A41 A42 A43 A44 B41 B42 Km Lm Rm f_ddt1 f_ddt2 f_dtau1

% build the state space system
A = [0 0 1 0; ...
    0 0 0 1; ...
    A31 A32 A33 A34; ...
    A41 A42 A43 A44];

B = [0; 0; B31; B41];
C = eye(4);
D = 0;

eigs = -5 * [1.1 1.2 1.3 1.4];
K = place(A, B, eigs);

% simulate nonlinear system (motor dynamics ignored) with a linear controller
tspan = [0, 4];
y0 = [0 1.2*pi 0 0];
ref = [0 pi 0 0];
[t, y] = ode45(@(t, y) odefun_torque(t, y, f_ddt1, f_ddt2, -K * (y - [0 pi 0 0]')), tspan, y0);
torque = (y - ref) * -K';

subplot(2,2,1)
plot(t, y(:,1)*180/pi, t, y(:,2)*180/pi), title("Arm positions"), xlabel("Time [s]"), ylabel("Anle [deg]"), grid on
subplot(2,2,[2,4])
plot(101.971 * torque, (60/(2*pi))*y(:,3)), title("Motor state"), xlabel("Torque [kgf-mm]"), ylabel("Speed [rpm]"), grid on
subplot(2,2,3)
plot(t, torque .* y(:,3)), title("Motor power"), xlabel("Time [s]"), ylabel("Power [Watt]"), grid on

sim('odefun_toruqe_sim');

% Pololu Items #3204, #4844 (34:1 Metal Gearmotor 25D HP 12V) Performance at 12 V


% A = [0 0 1 0 0; ...
%     0 0 0 1 0; ...
%     A31 A32 A33 A34 B31*Km; ...
%     A41 A42 A43 A44 B41*Km; ...
%     0 0 (-Km/Lm) 0 (-Rm/Lm)];
% 
% B = [0; 0; 0; 0; 1/Lm];
% C = [1 0 0 0 0;
%      0 0 0 0 0;
%      0 0 1 0 0;
%      0 0 0 0 0;
%      0 0 0 0 0];
% D = zeros(5,1);
% 
% obsv(A, C);

% eigs = eig(A);
% eigs(1) = -61;
% eigs(3) = -2; % dth1, this is the unstable one
% % eigs = [-3.1 -1.2 -1.3 -1.4 -1.5];
% K = place(A, B, eigs);
% % Q = diag([1 1 0.1 0.1 0.1]);
% % R = 1;
% % K = lqr(A, B, Q, R, 0);
% 
%
% 
% sys = ss(A, B, C, D, ...
%     'StateName', {'th1', 'th2', 'dth1', 'dth2', 'i'}, 'InputName', 'V', ...
%     'OutputName', {'th1', 'th2', 'dth1', 'dth2', 'i'});
% % % 
% % % t = linspace(0, 10, 1000);
% % % u = [1 * ones(1, length(t)/2), zeros(1, length(t)/2)];
% % % lsim(sys, u, t, [0 0 0 0 0])
% 
% % voltage_func = @(t) 0 + 10 * (t > 1 & t < 2);
% % voltage_func = @(t) (t < 2) * 10 * sin(10*t);
% tspan = [0:0.05:1];
% y0 = [0 0.8*pi 0 0 0];
% [t, y] = ode45(@(t, y) paper_odefun(t, y, -K * (y - [2; pi; 0; 0; 0])), tspan, y0);
% plot(t, y(:,1), t, y(:,2))

% C = [1 0 0 0 0];
% D = zeros(5,1);
% 
% Ai = [A zeros(length(A),1); -C 0];
% Bi = [B; 0];
% Ci = [C 0];
% Di = 0;
% 
% Q = diag([1 1 0.1 0.1 0.1 0.1]);
% R = 0.001;
% K = lqr(Ai, Bi, Q, R, 0);
% K(end) = -K(end);
% 
% Ai_cl = [A - B*K(1:(length(K)-1)) B*(K(length(K))); -C 0];
% Bi_cl = [zeros(length(A),1); 1];
% Ci_cl = Ci;
% Di_cl = 0;
% 
% tspan = [0:0.05:1];
% y0 = [0 0.8*pi 0 0 0];
% [t, y] = ode45(@(t, y) paper_odefun(t, y, -K * (y - [2; pi; 0; 0; 0])), tspan, y0);