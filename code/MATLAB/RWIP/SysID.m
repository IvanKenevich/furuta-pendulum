clc; clear; close all;
f = readmatrix('Frequency.txt');
w = f*2*pi';
M = readmatrix('Magnitude.txt')';

fun=@(x)bodeFit(x,w,M);
% A=[];
% b=[];
% Aeq=[];
% beq=[];
lb=zeros(1,3);
ub=30*ones(1,3);
x0=(ub-lb)/2;
x = fminsearch(fun,x0);
s=tf('s');
K=x(3);
z=x(1);
wn=x(2);
sys=K*wn^2/(s^2+2*z*wn*s+wn^2)

[Mag, Ph]=bode(sys,w); Mag=Mag(:)'; Mag=20*log10(Mag); Ph=Ph(:)';
figure
semilogx(f,Mag,'b')
hold on
semilogx(f,M,'ro')
title('Frequency Response')
xlabel('Frequency (Hz)')
ylabel('Magnitude (dB)')
set(gcf, 'Position',  [550, 350, 800, 400])
grid