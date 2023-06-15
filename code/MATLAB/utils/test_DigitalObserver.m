clear; clc; close all;
load("../paper/derived_systems.mat")








% % Full order Observer
% 
% xkm1_simulation        = zeros(Model_Order,1);
% xo_km1                 = zeros(Model_Order,1);
% Co_km1                 = 0.0;
% Rin(1:n_steps)         = 1.0;  %Step input
% 
% noise_C_magnitude = 0.005;                                      % Mesurment noise magnitude
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
%     F0            = Rin(ii) - K*xo_km1;  
%     xk_simulation = A*xkm1_simulation + B*F0;
%     
%     % Measurment Simulation (This would be directly measured in the arctual implimentation)
%     Co_k = [(xk_simulation(1) + noise_x1(ii))];  % Note Measurment comes from the simulation here
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
%     Co_save(ii)              = Co_km1;
% 
% end
% 
% x_simulation_FO_save_M  = cell2mat(x_simulation_FO_save);
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
% 