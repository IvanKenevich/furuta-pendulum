clear; close all; clc;

%% actual behavior

x = readmatrix("dataset1.txt");
x = x(474:883, :); % cut out the relevant portion
x(:,1) = 1e-6 * x(:,1); % microsecond to second conversion
x(:,1) = x(:,1) - x(1,1); % remove time offset

figure
hold on
plot(x(:,1), x(:,2) / 4 * 100 - 50)
plot(x(:,1), mod(x(:,3), 360))
plot(x(:,1), x(:,4))
hold off