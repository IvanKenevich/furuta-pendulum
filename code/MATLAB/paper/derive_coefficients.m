function [up_lin_coeff, dn_lin_coeff, f1, f2, f3] = derive_coefficients(simplify_inertia)
%DERIVE_COEFFICIENTS Summary of this function goes here
%   Detailed explanation goes here
syms J1 J2 real
syms J1xx J1xy J1xz J1yy J1yz J1zz real
syms J2xx J2xy J2xz J2yy J2yz J2zz real
syms t real
syms th1(t) th2(t)
syms l1 l2 L1 L2 positive
syms m1 m2 g positive
syms b1 b2 positive
syms tau1 tau2 real
syms t1 t2 dt1 dt2 ddt1 ddt2 real
syms Rm Lm Km V real

J1 = [J1xx, J1xy, J1xz;
      J1xy, J1yy, J1yz;
      J1xz, J1yz, J1zz];
J2 = [J2xx, J2xy, J2xz;
      J2xy, J2yy, J2yz;
      J2xz, J2yz, J2zz];
  
if simplify_inertia
    J1 = subs(J1, {J1xy, J1xz, J1yz}, [0, 0, 0]);
    J2 = subs(J2, {J2xy, J2xz, J2yz}, [0, 0, 0]);
end
  
R1 = [
    cos(th1), sin(th1), 0;
    -sin(th1), cos(th1), 0;
    0, 0, 1
];


R2 = subs(R1, th1, th2) * [0 0 -1; 0 1 0; 1 0 0];

w1 = [0; 0; diff(th1, t)];
v1 = [0; 0; 0];
v1c = v1 + cross(w1, [l1; 0; 0]);

w2 = R2 * w1 + [0; 0; diff(th2,t)];
v2 = R2 * cross(w1, [L1; 0; 0]);
v2c = v2 + cross(w2, [l2; 0; 0]);

Ep1 = 0;
Ek1 = (v1c.' * m1 * v1c + w1.' * J1 * w1) / 2;
Ep2 = g * m2 * l2 * (1 - cos(th2));
Ek2 = (v2c.' * m2 * v2c + w2.' * J2 * w2) / 2;
% % %                                      VV this is written as L2 in the
% % %                                      VV paper, but only L1 makes the difference zero
% % Ek2paper = (1/2) * diff(th1)^2 * (m2 * L1^2 + (m2 * l2^2 + J2yy)*sin(th2)^2 + J2xx * cos(th2)^2) ...
% %          + (1/2) * diff(th2)^2 * (J2zz + m2*l2^2) + m2*L1*l2*cos(th2)*diff(th1)*diff(th2);
% % simplify(Ek2 - Ek2paper)

Ep = Ep1 + Ep2;
Ek = Ek1 + Ek2;
L = Ek - Ep;

q = [th1; th2];
b = [b1; b2];
Q = [tau1; tau2];

el1 = diff(diff(L, diff(th1,t)), t);
el2 = diff(diff(L, diff(th2,t)), t);
el3 = -diff(L, th1);
el4 = -diff(L, th2);

% % el1paper = diff(th1,2) * (J1zz + m1*l1^2 + m2*L1^2 + (m2*l2^2+J2yy) * sin(th2)^2 + J2xx*cos(th2)^2) ...
% %     + m2*L1*l2*cos(th2)*diff(th2, 2) - m2*L1*l2*sin(th2)*diff(th2)^2 ...
% %     + diff(th1)*diff(th2)*sin(2*th2)*(m2*l2^2 + J2yy - J2xx);
% % el2paper = diff(th1,2)*m2*L1*l2*cos(th2) + diff(th2,2)*(J2zz + m2*l2^2) ...
% %     - diff(th1)*diff(th2)*m2*L1*l2*sin(th2);
% % el4paper = - (1/2) * diff(th1)^2 * sin(2*th2)*(m2*l2^2+J2yy-J2xx) ...
% %     + diff(th1)*diff(th2)*m2*L1*l2*sin(th2) + g*m2*l2*sin(th2);

eq_motion = [el1; el2] + b .* diff(q) + [el3; el4] == Q;

% % eq_motion_paper = [diff(th1,2) * (J1zz + m1*l1^2 + m2*L1^2 + (J2yy + m2*l2^2) ...
% %     * sin(th2)^2 + J2xx*cos(th2)^2) + diff(th2,2)*m2*L1*l2*cos(th2) ...
% %     - m2*L1*l2*sin(th2)*diff(th2)^2 + diff(th1)*diff(th2)*sin(2*th2) ...
% %     * (m2*l2^2 + J2yy - J2xx) + b1*diff(th1);
% %     
% %     diff(th1,2)*m2*L1*l2*cos(th2) + diff(th2,2)*(m2*l2^2+J2zz) ...
% %     + (1/2)*diff(th1)^2*sin(2*th2)*(-m2*l2^2-J2yy+J2xx) ...
% %     + b2*diff(th2) + g*m2*l2*sin(th2)] == Q;

syms I1 I2 positive

I1 = (J1yy + J1zz) / 2;
I2 = (J2yy + J2zz) / 2;

if simplify_inertia
    eq_motion = subs(eq_motion, {J1xx, J1yy, J1zz}, [0, I1, I1]);
    eq_motion = subs(eq_motion, {J2xx, J2yy, J2zz}, [0, I2, I2]);
end

eq_motion = subs(eq_motion, {th1, th2, diff(th1,t), diff(th2,t), diff(th1,t,2), diff(th2,t,2)}, ...
                            [t1, t2, dt1, dt2, ddt1, ddt2]);

c = children(eq_motion);
first = c{1};
second = c{2};
em1 = first{1} == first{2};
em2 = second{1} == second{2};

eq_motion = [em1; em2];
[f1, f2] = solve(eq_motion, [ddt1, ddt2]);
jac = jacobian([f1;f2], [t1, t2, dt1, dt2, tau1, tau2]);
up_lin_coeff = subs(jac, {t1, t2, dt1, dt2}, [0 pi 0 0]);
dn_lin_coeff = subs(jac, {t1, t2, dt1, dt2}, [0 0 0 0]);
% f3 = (Km/Lm) * V - (Rm/Lm) * tau1 - (1/Lm) * dt1 ;

% % syms I0h I1h I2h real
% % I1h = I1 + m1*l1^2;
% % I2h = I2 + m2*l2^2;
% % I0h = I1h + m2*L1^2;
% % A31p = 0; simplify(A31 == A31p, 100)
% % A32p = (g * m2^2 * l2^2 * L1) / (I0h * I2h - m2^2 * L1^2 * l2^2); simplify(A32 == A32p, 100)
% % A33p = (-b1 * I2h) / (I0h * I2h - m2^2 * L1^2 * l2^2); simplify(A33 == A33p, 100)
% % A34p = (-b2 * m2 * l2 * L1) / (I0h * I2h - m2^2 * L1^2 * l2^2); simplify(A34 == A34p, 100)
% % A41p = 0; simplify(A41 == A41p, 100)
% % A42p = (g*m2*l2*I0h) / (I0h*I2h - m2^2*L1^2*l2^2); simplify(A42 == A42p, 100)
% % A43p = (-b1*m2*l2*L1) / (I0h*I2h - m2^2 * L1^2 * l2^2); simplify(A43 == A43p, 100)
% % A44p = (-b2*I0h) / (I0h*I2h - m2^2 * L1^2 * l2^2); simplify(A44 == A44p, 100)
% % B31p = (I2h) / (I0h*I2h - m2^2 * L1^2 * l2^2); simplify(B31 == B31p, 100)
% % B41p = (m2*L1*l2) / (I0h*I2h - m2^2 * L1^2 * l2^2); simplify(B41 == B41p, 100)
% % B32p = (m2*L1*l2) / (I0h*I2h - m2^2 * L1^2 * l2^2); simplify(B32 == B32p, 100)
% % B42p = (I0h) / (I0h * I2h - m2^2 * L1^2 * l2^2); simplify(B42 == B42p, 100)
end

