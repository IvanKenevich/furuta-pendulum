function [out] = f_ddt2(dt1,dt2,t2,tau1,tau2)
%F_DDT2 Summary of this function goes here
%   Detailed explanation goes here
out = ((dt2.*7.008988189722898e-8-tau2.*2.503210067758178e-4+sin(t2).*9.777670206986761e-7-tau2.*cos(t2).^2.*7.790075654570001e-4+dt2.*sin(t2).^2.*2.29014289619162e-7-tau2.*sin(t2).^2.*8.179081772112928e-4+sin(t2).^3.*3.194792366559896e-6+cos(t2).^2.*sin(t2).*3.042844530666064e-6-dt1.*cos(t2).*8.447592950776101e-9+tau1.*cos(t2).*8.447592950776101e-5+dt2.*cos(t2).^2.*2.1812211832796e-7-dt1.^2.*cos(t2).*sin(t2).^3.*3.181712845235778e-8-dt1.^2.*cos(t2).^3.*sin(t2).*3.030387085749956e-8-dt1.^2.*cos(t2).*sin(t2).*9.737640298529774e-9+dt2.^2.*cos(t2).*sin(t2).*7.136182666200207e-9-dt1.*dt2.*cos(t2).^2.*sin(t2).*6.572330672728829e-9).*-1.0)./(cos(t2).^2.*2.33787992415382e-8+sin(t2).^2.*3.203878156838204e-8+9.805477291366021e-9);
end
