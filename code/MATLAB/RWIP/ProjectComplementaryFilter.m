clc; clear; close all; format long
N = 100;
Ts = 1440e-6;
Ord = 2;
s = tf('s');

G = (N/(s+N))^Ord;
GD = c2d(G,Ts)
[GD.num, GD.den]
GD2 = 1-GD
[GD2.num, GD2.den]