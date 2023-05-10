function [dydt] = paper_odefun(t, y, V)
% Equations of motion for the pendulum properties as listed in the paper
t1 = y(1);
t2 = y(2);
dt1 = y(3);
dt2 = y(4);
tau1 = y(5);
tau2 = 0;%0.2*rand() - 0.1; % no disturbance or rand()

ddt1 = ((dt1.*5.5028e-7-tau1.*5.5028e-3-dt2.^2.*sin(t2).*1.698054024e-5-cos(t2).*sin(t2).*3.360158478e-4-dt2.*cos(t2).*8.64024e-7+tau2.*cos(t2).*3.0858e-3+dt1.^2.*cos(t2).^2.*sin(t2).*1.698054024e-5+dt1.*dt2.*cos(t2).*sin(t2).*6.056161568e-5).*-1.0)./(cos(t2).^2.*2.2373718e-5+sin(t2).^2.*6.217668748e-5+1.7361334e-4);
ddt2 = ((dt2.*8.834e-6-tau2.*3.155e-2+sin(t2).*3.43551105e-3-tau2.*cos(t2).^2.*5.7963e-3+dt2.*sin(t2).^2.*3.163748e-6-tau2.*sin(t2).^2.*1.12991e-2+sin(t2).^3.*1.2303702981e-3+cos(t2).^2.*sin(t2).*6.311649033e-4-dt1.*cos(t2).*3.0858e-7+tau1.*cos(t2).*3.0858e-3+dt2.*cos(t2).^2.*1.622964e-6-dt1.^2.*cos(t2).*sin(t2).^3.*6.217668748e-5-dt1.^2.*cos(t2).^3.*sin(t2).*3.189587964e-5-dt1.^2.*cos(t2).*sin(t2).*1.7361334e-4+dt2.^2.*cos(t2).*sin(t2).*9.52216164e-6-dt1.*dt2.*cos(t2).^2.*sin(t2).*3.396108048e-5).*-1.0)./(cos(t2).^2.*2.2373718e-5+sin(t2).^2.*6.217668748e-5+1.7361334e-4);
dtau1 = 16 * V - 1560 * tau1 - 200 * dt1;

dydt = [dt1; dt2; ddt1; ddt2; dtau1];
end

