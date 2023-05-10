clc; clear; close all; format shortg;
%%
syms x1 x2 x3 x4 x1m1 x2m1 x3m1 x4m1 kmgl Iso bb kt ke R br u Ir Ts;

eqn1 = (x1-x1m1)/Ts == x2;
eqn2 = (x2-x2m1)/Ts == (kmgl*x1 - bb*x2 + (kt*ke/R + br)*x3 - kt/R*u)/Iso;
eqn3 = Ir*((x3-x3m1)/Ts + (x2-x2m1)/Ts) == kt/R*u - (kt*ke/R + br)*x3;
eqn4 = (x4-x4m1)/Ts == x3;

eqns = [eqn1; eqn2; eqn3; eqn4];
vars = [x1; x2; x3; x4];

solns = solve(eqns,vars);
%%

lr = 10e-2; %m
mb = 14.685e-3; %kg
nBolts = 20; %number of bolts/nuts on the wheel rim
mr = (35.4 ... % printed wheel
      + 10 ... % shaft coupler
      + nBolts*(3.166667+0.66) ... % bolts + nuts on wheel
      + 4*(0.32+0.925))*1e-3; % bolts and nuts on shaft coupler [kg]
Ir = (667.94676 ... % printed wheel
      + 399.22 ... % shaft coupler
      + 0.8^2*(0.32+0.925)*4 ... % bolts and nuts on shaft coupler
      + 4.5^2*(3.166667+0.66)*nBolts)*1e-7; % bolts + nuts on wheel [kg*m^2]
Iro = mr*lr^2;
mm = 97e-3; %kg
lm = lr;
lb = lr/2;
Imo = mm*lm^2;
Ibo = mb*lb^2;
g = 9.81; %m/s^2 %%% LOOK AT ME, TRY TO SET ME TO NEGATIVE, WE'RE NOT SURE IF THIS IS RIGHT
Iso = Imo + Iro + Ibo;
kmgl = (mb*lb + mm*lm + mr*lr)*g;


ke = 0.069963; %%% LOOK AT ME, MATLAB CAME UP WITH SLIGHTLY DIFFERENT NUMBERS THAN THESE, GO TO MOTORCONSTANTSCALC.M and SOLVE THOSE EQUATIONS WITH YOUR CALCULATOR TO CONFIRM
kt = ke; %N*m/A
L = 0.593368;
R = 9;

bb = 0.024; %N*m*s
br = 0.001426; %N*m*s

Ts = 500e-6*4;

Vmax = 12;
%%
solns = subs(solns,{'Ir' 'Iso' 'R' 'ke' 'kt' 'br' 'bb' 'kmgl'}, {Ir Iso R ke kt br bb kmgl});

x1 = vpa(solns.x1,6)
x2 = vpa(solns.x2,6)
x3 = vpa(solns.x3,6)
x4 = vpa(solns.x4,6)

%%
attemptedAngle = 10; %deg

PMO = 5;
tr = 0.4;
Tfinal = 2;

A = [0 1 0; 
    kmgl/Iso -bb/Iso kt*ke/(R*Iso)+br/Iso; 
    -kmgl/Iso bb/Iso -(Iso+Ir)/(Iso*Ir)*(br+ke*kt/R)];
B = [0; -kt/(R*Iso); (Iso+Ir)/(Iso*Ir)*kt/R];
C = eye(3);
D = [0;0;0];

sys = ss(A,B,C,D);
sysD = c2d(sys,Ts);

z=((-log(PMO/100))/(sqrt(log(PMO/100)^(2)+pi^(2))));
wn=((2.917*(z^(2)-0.142852245458*z+0.342817963661))/(tr));

p1=-z*wn+wn*sqrt(1-z^2)*1i;
p2=-z*wn-wn*sqrt(1-z^2)*1i;
p3=5*(floor(real(p1))-1);
p = [p1, p2, p3];
pd=[exp(p(1)*Ts) exp(p(2)*Ts) exp(p(3)*Ts)];
K = place(sysD.A,sysD.B,pd);

% Q = [1  0  0;
%      0  1  0; 
%      0  0  1]; %Performance weight
% R = 1e2; %Control effort weight
% [K, ~, ~] = lqr(sysD,Q,R);

AD = sysD.A;
BD = sysD.B;

sprintf('#define IntervalMicro %g',Ts*1e6)
sprintf('a11 = %g, a12 = %g, a13 = %g,\na21 = %g, a22 = %g, a23 = %f,\na31 = %g, a32 = %g, a33 = %g;', AD(1,1), AD(1,2), AD(1,3), AD(2,1), AD(2,2), AD(2,3), AD(3,1), AD(3,2), AD(3,3))
sprintf('b1 = %g, b2 = %g, b3 = %g;',BD(1), BD(2), BD(3))
sprintf('k1 = %g, k2 = %g, k3 = %g;',K(1), K(2), K(3))

Acl = sysD.A - sysD.B*K;
sysG = ss(Acl,sysD.B,sysD.C,sysD.D,Ts);

T = 0:Ts:Tfinal;
U = [0,zeros(1,length(T)-1)];
X0 = [deg2rad(attemptedAngle); 0; 0];

[y, t]=lsim(sysG,U,T,X0);
% [y, t]=lsim(sysD,U,T,X0);

subplot(311)
plot(t,rad2deg(y(:,1)))
ylabel('\theta (°)','FontSize',14)
subplot(312)
plot(t,rad2deg(y(:,2)))
ylabel('$\dot\theta\ (^{\circ}/s)$','Interpreter','latex','FontSize',14)
subplot(313)
plot(t,y(:,3))
ylabel('ω (rad/s)','FontSize',14)

figure
sim=sim('StateSpace');
simt=sim.out(:,1);
out1=sim.out(:,2);
out2=sim.out(:,3);
out3=sim.out(:,4);
subplot(311)
plot(simt,rad2deg(out1))
ylabel('θ (°)','FontSize',14)
subplot(312)
plot(simt,rad2deg(out2))
ylabel('$\dot\theta\ (^{\circ}/s)$','Interpreter','latex','FontSize',14)
subplot(313)
plot(simt,out3)
ylabel('ω (rad/s)','FontSize',14)


%% friction
% 
% theta = readmatrix("Friction of Arm Bearing.txt")';
% theta = theta(260:440);
% theta = theta - mean(theta);
% theta = abs(theta);
% 
% Ts = 50e-3;
% t = Ts*(1:length(theta));
% 
% [pks,locs] = findpeaks(theta,t);
% wd = pi/mean(diff(locs));
% 
% for i=1:length(locs)-1
%     x1 = pks(i);
%     x2 = pks(i+1);
%     delta = log(x1/x2);
%     z(i) = delta/(2*pi);
% end
% 
% z = mean(z);
% 
% wn = wd/sqrt(1-z^2);
% 
% b = Iso*2*z*wn;
% 
% figure
% plot(t,theta,locs,pks,'ro')
% xlabel('Time (s)')
% ylabel('Amplitude (°)')