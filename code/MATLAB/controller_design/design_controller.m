clear; clc; close all;
load("../paper/derived_systems.mat")

sys = systems.full;

A = sys.A;
B = sys.B;

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
K = place(A, B, p);

% simulate nonlinear system (motor dynamics ignored) with a linear controller
tspan = [0, 5];
y0 = [0 deg2rad(165) 0 0 0];
% ref = [0 pi 0 0];
[t, y] = ode45(@(t, y) sys_odefun(t, y, f_ddt1, f_ddt2, - K * (y - [y(1) pi 0 0 0]'), motor_pars), tspan, y0);
% torque = (y - ref) * -K';

for row=1:height(y)
    yrow = y(row, :);
    
    V(row) = - K * (yrow - [yrow(1) pi 0 0 0])';
    if abs(V(row)) > motor_pars.Vmax
        V(row)= sign(V(row)) * 12;
    end
end

subplot(2,1,1)
plot(t, y(:,1)*180/pi, t, y(:,2)*180/pi), title("Arm positions"), xlabel("Time [s]"), ylabel("Anle [deg]"), ylim([180 - 360, 180 + 360]), grid on
subplot(2,1,2)
plot(t, y(:,3)*180/pi/360, t, y(:,4)*180/pi/360), title("Arm velocities"), xlabel("Time [s]"), ylabel("Velocity [rps]"), grid on