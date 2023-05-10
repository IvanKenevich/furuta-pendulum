clc; clear; close all;

Ts = 1/100;
Tend = 5;
t = 0:Ts:Tend;
nSamples = length(t);

wx = zeros(1,nSamples);
wy = wx; wz = wx;
thx = wx; thy = wx; thz = wx;

tx = 500*ones(1,nSamples); ty = wx; tz = wx;


thx(1) = deg2rad(5);

for i = 2:nSamples
    wx(i) = 0.002777*Ts*wx(i-1)^2 - 0.3891*Ts*wx(i-1)*wy(i-1) + 0.04918*Ts*wx(i-1)*wz(i-1) + wx(i-1) - 0.04918*Ts*wy(i-1)^2 - 0.2244*Ts*wy(i-1)*wz(i-1) + 0.0464*Ts*wz(i-1)^2 + 2.901e-5*Ts*tx(i-1) - 7.435e-7*Ts*ty(i-1) - 9.159e-6*Ts*tz(i-1);
    wy(i) = 0.3544*Ts*wx(i-1)^2 + 0.04362*Ts*wy(i-1)*wx(i-1) - 0.3544*Ts*wz(i-1)^2 - 0.04362*Ts*wy(i-1)*wz(i-1) + wy(i-1) - 7.435e-7*Ts*tx(i-1) + 2.913e-5*Ts*ty(i-1) - 7.435e-7*Ts*tz(i-1);
    wz(i) = - 0.0464*Ts*wx(i-1)^2 + 0.2244*Ts*wx(i-1)*wy(i-1) - 0.04918*Ts*wx(i-1)*wz(i-1) + 0.04918*Ts*wy(i-1)^2 + 0.3891*Ts*wy(i-1)*wz(i-1) - 0.002777*Ts*wz(i-1)^2 + wz(i-1) - 9.159e-6*Ts*tx(i-1) - 7.435e-7*Ts*ty(i-1) + 2.901e-5*Ts*tz(i-1);
    thx(i) = wx(i)*Ts + thx(i-1);
    thy(i) = wy(i)*Ts + thy(i-1);
    thz(i) = wz(i)*Ts + thz(i-1);
end

X = [thx;thy;thz;wx;wy;wz];

plot(t,X)
legend('thx','thy','thz','wx','wy','wz')