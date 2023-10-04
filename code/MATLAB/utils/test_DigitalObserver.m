% clear; clc; close all;
% 
% Ts = 1e-3;
% s = tf('s');
% sys_tf = ss(1 / (s^2 + 3*s + 7));
% sysD = c2d(sys_tf, Ts);
% 
% C = sys_tf.C;
% 
% p = [-1, -1.1];
% pd = exp(p * Ts);
% K = place(sys_tf.A, sys_tf.B, p);
% Kd = place(sysD.A,sysD.B,pd);
% 
% sys_cl = sys_tf;
% sys_cl.A = sys_tf.A - sys_tf.B * K;
% sys_cl_D = c2d(sys_cl, Ts);
% 
% sys_obs = Observer(sysD, Kd, p(2), 8, true, Ts);
% 
% t = 0:Ts:12;
% [y_c, ~, x_c] = initial(sys_cl_D,ones(size(C)), t);
% [y_o, ~, x_o] = initial(sys_obs,[ones(size(C)),zeros(size(C))], t);  
% figure
% hold on
% plot(t,x_c,'r--', t, x_o(:,3:end),'k--', t, x_o(:,1:2), 'b--')
% title('Free Response with and without Observer')
% subtitle('Different ICs')
% ylabel('Amplitude')
% xlabel('Time (s)')
% legend('Without Observer','', 'Observed states', '', 'With observer', '')
% hold off
% 
% y_err = abs(y_o-y_c);
% figure
% plot(t,y_err)
% title('Observer Error')
% ylabel('Error')
% xlabel('Time (s)')
% hold on
% yline(0.1,'k--')
% xline(2,'k--')

clear; clc; close all;
load("../controller_design/controller_parameters.mat")

A = sysD.A;
B = sysD.B;
C = sysD.C;
D = sysD.D;

% simulation parameters
t_end = 10; % s
dt = 1e-3;
t = 0:dt:t_end;
Vmax = 12;
% reference = [0 pi 0 0 0]';

% initialize states and estimates
x0 = [0 deg2rad(5) 0 0 0]';
xhat0 = [0 0 0 0 0]';
y0 = xhat0(1:end-1);
u0 = 0;

x = x0;
xhat = xhat0;
y = y0;
u = u0;

% for duration of simulation
for i = 2:length(t)
%   calculate controller input based on xhat
    u(i) = - Kd * xhat(:,i-1) + dist_mag * dist_time(i);
    if abs(u(i)) > Vmax
        u(i) = Vmax * sign(u(i));
    end
%   evolve system state based on previous state
    x(:,i) = A * x(:, i-1) + B * u(i);
    y(:,i) = C * x(:, i);
%   evolve system estimate based on measurement and input and previous xhat
    xhat(:,i) = A * xhat(:, i-1) + B * u(i-1) + L * (y(:, i-1) - C * xhat(:, i-1));
end

figure
ax1 = subplot(2,1,1);
plot(t, rad2deg(x(1,:)));
ax2 = subplot(2,1,2);
plot(t, rad2deg(x(2,:)), t, rad2deg(xhat(2,:)));
linkaxes([ax1 ax2], 'x')
% ylim([-10 10]);