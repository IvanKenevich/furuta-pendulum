clear; close all; clc;

% get the symbolically derived coefficients and functions
[up_lin_coeff, dn_lin_coeff, f1, f2] = derive_coefficients(false);

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
run("sys_props.m");

cf = up_lin_coeff; % select the linearization about the up position
cf = double(subs(cf));

[A31, A32, A33, A34, B31, B32, A41, A42, A43, A44, B41, B42] = deal(...
    cf(1,1), cf(1,2), cf(1,3), cf(1,4), cf(1,5), cf(1,6), ...
    cf(2,1), cf(2,2), cf(2,3), cf(2,4), cf(2,5), cf(2,6));

f_ddt1 = matlabFunction(vpa(subs(f1)));
f_ddt2 = matlabFunction(vpa(subs(f2)));

% build the state space system - no motor dynamics, perfect torque source
A = [0 0 1 0; ...
    0 0 0 1; ...
    A31 A32 A33 A34; ...
    A41 A42 A43 A44];

B = [0; 0; B31; B41];
C = eye(4);
D = 0;
systems.nomotor = ss(A, B, C, D);

% motor dynamics included, but inductance is assumed to be 0 - no observer
% necessary for current
A = [0 0 1 0
    0 0 0 1
    A31 A32 (A33 - B31 * (Km/Rm)) A34
    A41 A42 (A43 - B41 * (Km/Rm)) A44];

B = [0; 0; (B31 * (Km^2/Rm)); (B41 * (Km^2/Rm))];
C = eye(4);
D = 0;
systems.noinductance = ss(A, B, C, D);

% motor dynamics included, with inductance, observer is needed for current
A = [0 0 1 0 0
    0 0 0 1 0
    A31 A32 A33 A34 B31 * Km
    A41 A42 A43 A44 B41 * Km
    0 0 (-Km/Lm) 0 (-Rm/Lm)];

B = [0; 0; 0; 0; (1/Lm)];
C = eye(5);
D = 0;
systems.full = ss(A, B, C, D);

C = eye(4,5); % realistic measurement matrix, we don't measure current
% C = eye(2,5); % set to this if you wish to observe velocity too
systems.full_measured = ss(A, B, C, D);

motor_pars.Km = Km;
motor_pars.Lm = Lm;
motor_pars.Rm = Rm;
motor_pars.Vmax = Vmax;

sys_odefun = @sys_odefun;

% clearvars -except systems motor_pars
clearvars -except f_ddt1 f_ddt2 systems motor_pars sys_odefun
save("derived_systems.mat")