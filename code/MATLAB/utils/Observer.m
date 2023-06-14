function [sys_obs, L] = Observer(sys_ss, K, p_dom_cont, obs_fact, is_digital, Ts)
%OBSERVER Design observer
%   sys_ss - 
%   If there is an issue placing observer poles, try changing pole spacing
A = sys_ss.A;
B = sys_ss.B;
C = sys_ss.C;

N_x = height(A);

p_o = obs_fact*real(p_dom_cont)* (1 + (0:N_x-1) * 0.05); % place observer poles much faster than system poles, space them out
if is_digital
    p_o = exp(p_o * Ts);
end
L = place(A', C', p_o);
L = L';

A_o = [  A,  -B*K
       L*C,  A-B*K-L*C];
B_o = [B 
       B];
C_o = [zeros(height(C), height(A)), C];
D_o = 0;

sys_obs = ss(A_o,B_o,C_o,D_o);

end

