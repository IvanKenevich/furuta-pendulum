clear; clc; close all;

s = tf('s');
sys_tf = ss(1 / (s^2 + 3*s + 7));

C = sys_tf.C;

p = [-1, -1.1];
K = place(sys_tf.A, sys_tf.B, p);

sys_cl = sys_tf;
sys_cl.A = sys_tf.A - sys_tf.B * K;

sys_obs = Observer(sys_tf, K, p(2), 8);

t = linspace(0, 12, 1000);
[y_c, ~] = initial(sys_cl,ones(size(C)), t);
[y_o, ~] = initial(sys_obs,[ones(size(C)),zeros(size(C))], t);  
figure
hold on
plot(t,y_c,t,y_o,'--')
title('Free Response with and without Observer')
subtitle('Different ICs')
ylabel('Amplitude')
xlabel('Time (s)')
legend('Without Observer','With Observer')
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