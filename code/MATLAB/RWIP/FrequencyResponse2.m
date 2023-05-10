clc; close all;
% Freq01 = xlsread('Frequency Response 2.0.xlsx','0.1 Hz');
% Freq02 = xlsread('Frequency Response 2.0.xlsx','0.2 Hz');
% Freq04 = xlsread('Frequency Response 2.0.xlsx','0.4 Hz');
% Freq07 = xlsread('Frequency Response 2.0.xlsx','0.7 Hz');
% Freq1 = xlsread('Frequency Response 2.0.xlsx','1 Hz');
% Freq2 = xlsread('Frequency Response 2.0.xlsx','2 Hz');
% Freq4 = xlsread('Frequency Response 2.0.xlsx','4 Hz');
% Freq7 = xlsread('Frequency Response 2.0.xlsx','7 Hz');

% Freq02 = Freq02(1:9/10.4*length(Freq02),:);
% Freq04 = Freq04(1:6/8.1*length(Freq04),:);
% Freq07 = Freq07(1:3.2/4.6*length(Freq07),:);
% Freq1 = Freq1(1:3.2/3.7*length(Freq1),:);
% Freq2 = Freq2(2:3.2/3.7*length(Freq2),:);
% Freq4 = Freq4(1:2.4/3.6*length(Freq4),:);
% Freq7 = Freq7(1:1.75/3*length(Freq7),:);
% Freq2 = Freq2(1:2.44/3.12*length(Freq2),:);
% Freq4 = Freq4(1:1.75/2*length(Freq4),:);
% Freq7 = Freq7(1:1.62/1.75*length(Freq7),:);

% Vin01 = abs(Freq01(:,1));
% Vin02 = abs(Freq02(:,1));
% Vin04 = abs(Freq04(:,1));
% Vin07 = abs(Freq07(:,1));
% Vin1 = abs(Freq1(:,1));
% Vin2 = abs(Freq2(:,1));
% Vin4 = abs(Freq4(:,1));
% Vin7 = abs(Freq7(:,1));
% 
% Vout01 = abs(Freq01(:,2));
% Vout02 = abs(Freq02(:,2));
% Vout04 = abs(Freq04(:,2));
% Vout07 = abs(Freq07(:,2));
% Vout1 = abs(Freq1(:,2));
% Vout2 = abs(Freq2(:,2));
% Vout4 = abs(Freq4(:,2));
% Vout7 = abs(Freq7(:,2));
% 
% t01 = 400e-6*(1:length(Freq01));
% t02 = 400e-6*(1:length(Freq02));
% t04 = 400e-6*(1:length(Freq04));
% t07 = 400e-6*(1:length(Freq07));
% t1 = 400e-6*(1:length(Freq1));
% t2 = 400e-6*(1:length(Freq2));
% t4 = 400e-6*(1:length(Freq4));
% t7 = 400e-6*(1:length(Freq7));


% [Inpks01, Inlocs01] = findpeaks(Vin01,t01,'MinPeakDistance',1/0.1/2*0.9);
% [Inpks02, Inlocs02] = findpeaks(Vin02,t02,'MinPeakDistance',1/0.2/2*0.9);
% [Inpks04, Inlocs04] = findpeaks(Vin04,t04,'MinPeakDistance',1/0.4/2*0.9);
% [Inpks07, Inlocs07] = findpeaks(Vin07,t07,'MinPeakDistance',1/0.7/2*0.9);
% [Inpks1, Inlocs1] = findpeaks(Vin1,t1,'MinPeakDistance',1/1/2*0.9);
% [Inpks2, Inlocs2] = findpeaks(Vin2,t2,'MinPeakDistance',1/2/2*0.9);
% [Inpks4, Inlocs4] = findpeaks(Vin4,t4,'MinPeakDistance',1/4/2*0.9);
% [Inpks7, Inlocs7] = findpeaks(Vin7,t7,'MinPeakDistance',1/7/2*0.9);
% 
% [Outpks01, Outlocs01] = findpeaks(Vout01,t01,'MinPeakDistance',1/0.1/2*0.9);
% [Outpks02, Outlocs02] = findpeaks(Vout02,t02,'MinPeakDistance',1/0.2/2*0.9);
% [Outpks04, Outlocs04] = findpeaks(Vout04,t04,'MinPeakDistance',1/0.4/2*0.9);
% [Outpks07, Outlocs07] = findpeaks(Vout07,t07,'MinPeakDistance',1/0.7/2*0.9);
% [Outpks1, Outlocs1] = findpeaks(Vout1,t1,'MinPeakDistance',1/1/2*0.9);
% [Outpks2, Outlocs2] = findpeaks(Vout2,t2,'MinPeakDistance',1/2/2*0.9);
% [Outpks4, Outlocs4] = findpeaks(Vout4,t4,'MinPeakDistance',1/4/2*0.9);
% [Outpks7, Outlocs7] = findpeaks(Vout7,t7,'MinPeakDistance',1/7/2*0.9);

% figure
% plot(t01,Vin01,Inlocs01,Inpks01,'o',t01,Vout01,Outlocs01,Outpks01,'o')
% figure
% plot(t02,Vin02,Inlocs02,Inpks02,'o',t02,Vout02,Outlocs02,Outpks02,'o')
% figure
% plot(t04,Vin04,Inlocs04,Inpks04,'o',t04,Vout04,Outlocs04,Outpks04,'o')
% figure
% plot(t07,Vin07,Inlocs07,Inpks07,'o',t07,Vout07,Outlocs07,Outpks07,'o')
% figure
% plot(t1,Vin1,Inlocs1,Inpks1,'o',t1,Vout1,Outlocs1,Outpks1,'o')
% figure
% plot(t2,Vin2,Inlocs2,Inpks2,'o',t2,Vout2,Outlocs2,Outpks2,'o')
% figure
% plot(t4,Vin4,Inlocs4,Inpks4,'o',t4,Vout4,Outlocs4,Outpks4,'o')
% figure
% plot(t7,Vin7,Inlocs7,Inpks7,'o',t7,Vout7,Outlocs7,Outpks7,'o')

% Mag01 = 20*log10(mean(Outpks01)/mean(Inpks01));
% Mag02 = 20*log10(mean(Outpks02)/mean(Inpks02));
% Mag04 = 20*log10(mean(Outpks04)/mean(Inpks04));
% Mag07 = 20*log10(mean(Outpks07)/mean(Inpks07));
% Mag1 = 20*log10(mean(Outpks1)/mean(Inpks1));
% Mag2 = 20*log10(mean(Outpks2)/mean(Inpks2));
% Mag4 = 20*log10(mean(Outpks4)/mean(Inpks4));
% Mag7 = 20*log10(mean(Outpks7)/mean(Inpks7));

Freq = [0.1;0.2;0.4;0.7;1;2;4;7];
Mag = [Mag01;Mag02;Mag04;Mag07;Mag1;Mag2;Mag4;Mag7];
writematrix(Mag,'Magnitude.txt')
writematrix(Freq,'Frequency.txt')

figure
semilogx(Freq,Mag,'-o')

% Ph01 = -180+mean(Outlocs01-Inlocs01)*0.1*360
% Ph02 = mean(Outlocs02(1:3)-Inlocs02)*0.2*360
% Ph04 = -180+mean(Outlocs04-Inlocs04)*0.4*360
% Ph07 = mean(Outlocs07(1:4)-Inlocs07)*0.7*360
% Ph1 = mean(Outlocs1(1:6)-Inlocs1)*1*360
% Ph2 = mean(Outlocs2-Inlocs2)*2*360
% Ph4 = mean(Outlocs4-Inlocs4)*4*360
% Ph7 = -mean(Outlocs7-Inlocs7)*7*360
% 
% Ph = -180-[Ph01;Ph02;Ph04;Ph07;Ph1;Ph2;Ph4;Ph7];
% 
% figure
% semilogx(Freq,Ph,'-o')
