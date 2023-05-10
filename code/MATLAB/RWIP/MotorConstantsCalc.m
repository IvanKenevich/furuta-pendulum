clc; clear; close all;
% 
%           529.3
%   ---------------------
%   s^2 + 21.57 s + 134.1

I = 2.620154095000000e-04; % [m2*kg]
R = 9; % Ohm

syms L Kt Ke b


e1 = I * L / Kt == (1 / 529.3);
e2 = (b * L + I * R) / Kt == (21.57 / 529.3);
e3 = (R * b + Kt * Ke) / Kt == (134.1 / 529.3);
e4 = Kt == Ke;

eqns = [e1; e2; e3; e4];

solns = solve(eqns);