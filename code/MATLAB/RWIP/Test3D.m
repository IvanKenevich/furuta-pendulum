clc; clear; close all;

Ixx = 38335.28;	Ixy = -1288.36;	Ixz = -12135.85;
Iyx = -1288.36;	Iyy = 34399.09;	Iyz = -1288.36;
Izx = -12135.85;	Izy = -1288.36;	Izz = 38335.28;

syms wx wy wz wxp1 wyp1 wzp1 Ts tx ty tz wxm1 wym1 wzm1;

wxp = (wxp1-wx)/Ts;
wyp = (wyp1-wy)/Ts;
wzp = (wzp1-wz)/Ts;
vars = [wxp1;wyp1;wzp1];

% wxp = (wx-wxm1)/Ts;
% wyp = (wy-wym1)/Ts;
% wzp = (wz-wzm1)/Ts;
% vars = [wx;wy;wz];

eqn1 = tx == Ixx*wxp - (Iyy-Izz)*wy*wz - Ixy*(wyp-wz*wx) - Iyz*(wy^2-wz^2) - Izx*(wzp+wx*wy);
eqn2 = ty == Iyy*wyp - (Izz-Ixx)*wz*wx - Iyz*(wzp-wx*wy) - Izx*(wz^2-wx^2) - Ixy*(wxp+wy*wz);
eqn3 = tz == Izz*wzp - (Ixx-Iyy)*wx*wy - Izx*(wxp-wy*wz) - Ixy*(wx^2-wy^2) - Iyz*(wyp+wz*wx);

eqns = [eqn1;eqn2;eqn3];

solns = solve(eqns,vars);

wxp1 = vpa(solns.wxp1,4)
wyp1 = vpa(solns.wyp1,4)
wzp1 = vpa(solns.wzp1,4)