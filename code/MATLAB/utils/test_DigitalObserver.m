clear; clc; close all;

Ts = 1e-3;
s = tf('s');
sys_tf = ss(1 / (s^2 + 3*s + 7));
sysD = c2d(sys_tf, Ts);

C = sys_tf.C;

p = [-1, -1.1];
pd = exp(p * Ts);
K = place(sys_tf.A, sys_tf.B, p);
Kd = place(sysD.A,sysD.B,pd);

sys_cl = sys_tf;
sys_cl.A = sys_tf.A - sys_tf.B * K;
sys_cl_D = c2d(sys_cl, Ts);

sys_obs = Observer(sysD, Kd, p(2), 8, true, Ts);

t = 0:Ts:12;
[y_c, ~, x_c] = initial(sys_cl_D,ones(size(C)), t);
[y_o, ~, x_o] = initial(sys_obs,[ones(size(C)),zeros(size(C))], t);  
figure
hold on
plot(t,x_c,'r--', t, x_o(:,3:end),'k--', t, x_o(:,1:2), 'b--')
title('Free Response with and without Observer')
subtitle('Different ICs')
ylabel('Amplitude')
xlabel('Time (s)')
legend('Without Observer','', 'Observed states', '', 'With observer', '')
hold off

y_err = abs(y_o-y_c);
figure
plot(t,y_err)
title('Observer Error')
ylabel('Error')
xlabel('Time (s)')
hold on
yline(0.1,'k--')
xline(2,'k--')

% clear; clc; close all;
% load("../controller_design/controller_parameters.mat")
% 
% A = sysD.A;
% B = sysD.B;
% D = sysD.D;
% 
% % Full order Observer
% n_steps = 1000;
% time    = (0:1:(n_steps-1))*Ts;
% xkm1_simulation        = zeros(5,1);
% xo_km1                 = zeros(5,1);
% Co_km1                 = zeros(4,1);
% Rin(1:n_steps)         = 1.0;  %Step input
% 
% noise_C_magnitude = 0;                                      % Mesurment noise magnitude
% noise_x1          = noise_C_magnitude*normrnd(0,1,1,n_steps);   % Noise defined as a normal distribution
% 
% % Simulate the closed loop response with an observer using recursive equations
% 
% for ii=1:n_steps
% 
%     % Save data for ploting
%     xo_save{ii}              = xo_km1;
%     x_simulation_FO_save{ii} = xkm1_simulation;  % Save Simulation SV's
% 
%     % Simulation Model (system states estimated from the theoritical model)
%     F0            = Rin(ii) - Kd*xo_km1;  
%     xk_simulation = A*xkm1_simulation + B*F0;
% 
%     % Measurment Simulation (This would be directly measured in the arctual implimentation)
%     Co_k = [(xk_simulation(1:4) + noise_x1(ii))];  % Note Measurment comes from the simulation here
% 
%     % Recursive estimation of the closed loop states using a full order observer
%     xo_k   = (A - B*K - L*D)*xo_km1 + B*Rin(ii) + L*Co_km1;
% 
%     % Update the System States
%     xo_km1 = xo_k;
% 
%     % Update Output
%     Co_km1 = Co_k;
% 
%     % Update simulation
%     xkm1_simulation          = xk_simulation;  % Update Simulation Model
%     Co_save{ii}              = Co_km1;
% 
% end
% 
% x_simulation_FO_save_M  = cell2mat(x_simulation_FO_save);
% 
% 
% [step_pos_cl,step_time_cl] = step(sys_cl_D,time);
% 
% figure
% stairs(step_time_cl,step_pos_cl)
% hold on
% stairs(time,x_simulation_FO_save_M(1,:))
% title({'Closed Loop Position step response - No matrix multiplications - Observer','Standard State Space Controller - With Observer'})
% xlabel('Time (sec)')
% ylabel('Position (inch)')
% legend('Step function','Recursive equation - Observer')
% xlim([0 2]);
% ylim([0 1.2]);
% 
% figure
% stairs(time,Co_save)
% title({'Measurement with noise','Standard State Space Controller - With Observer'})
% xlim([0 2]);
% ylim([0 1.2]);
