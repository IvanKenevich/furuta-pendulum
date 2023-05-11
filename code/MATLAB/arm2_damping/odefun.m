function [dydt] = odefun(t, y, params)

% I theta'' + b theta' + M L g sin(theta) = 0

% make these inputs in the future
I = params.I; % kg m^2
b = params.b; % N m s
M = params.M; % kg
L = params.L;
g = params.g; % m / s^2

dtheta = y(2);
theta = y(1);

dydt = [dtheta; ...
        (- b * dtheta - M * L * g * sin(theta)) / I];
end

