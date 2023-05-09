clear; clc; close all;
% Amplifier characteristics
% @10 Hz:
Kamp = 7.84 / 3.96;
Ktach = 0.02;
% @150 kHz: signal shape begins distorting,
%           but the gain is basically the same,
%           minimal phase offset
% after that, distortion increases, out amp goes up then down

% freq, amp_in, amp_out, out_t - in_t
data_motor = [...
    0.1 9 7.52 32e-3;
    0.5 9 7.36 20e-3;
    1 3.96                   7.20 18.4e-3;
    2 3.96 6.96 15.2e-3;
    4 3.96 6.48 16.6e-3;
    8 3.96 6.48 15.6e-3;
];

%plot_data(data_motor, "Experimental motor data");

% white-box model fitting
Ktotal = data_motor(1, 3) / data_motor(1, 2);
data_motor_norm = data_motor;
data_motor_norm(:, 3) = data_motor_norm(:, 3) / Ktotal;

zeta = 1;%1;
omega_n = 655;%150;
T = 100;
s = tf('s');
sys_exp = Ktotal * (omega_n ^ 2) / (s^2 + (2 * zeta * omega_n) * s + omega_n^2) * T / (s + T);
plot_data(data_motor, "Experimental Model", sys_exp);

% datasheet model
Kt = 1.82e-2;
Kbac = 1.82e-2;
I = 4.2e-6;
L = 0.63e-3; 
b = 2.6e-6;
R = 0.83;

sys_datasheet = (Kamp * Ktach * Kt) /...
                ((I*L)*s^2 + (b*L + I*R)*s + (R*b + Kt*Kbac));

plot_data(data_motor, "Datasheet Model", sys_datasheet);

%%% step response verification
step_data = readmatrix("NewFile1.csv");
[t, in, out] = deal(step_data(:,1), step_data(:,2), step_data(:,3));

Fs = 1 / (0.73e-4);
[ins, ts] = resample(in, t, Fs);
[outs, ts] = resample(out, t, Fs);

Y_exp = lsim(sys_exp, ins, ts);
Y_datasheet = lsim(sys_datasheet, ins, ts);

figure
plot(ts, ins, ts, outs, ts, Y_exp, ts, Y_datasheet, 'LineWidth', 2)
legend('Input', 'Output', 'Bode model', 'Datasheet model');
xlabel('Time [s]'); ylabel('Voltage [V]'); title("Step Response Verification");

%% P controller design
close all;

t_r = 0.003;
PMO = 25;
P_zeta = fzero(@(z) 100*exp(-pi*z/sqrt(1-z^2)) - PMO, 0.5);
P_omega_n = (1 - 0.4167 * P_zeta + 2.917 * P_zeta^2) / t_r;
P_Kp = 2;
figure
hold on
plot(-P_omega_n * P_zeta, P_omega_n * sqrt(1 - P_zeta^2), '*', 'MarkerSize', 24);
rlocus(sys_exp)
hold off

sys_P_cl = feedback(P_Kp * sys_exp, 1);
sys_P_eff = (1 - P_Kp * sys_exp) * P_Kp * 4;
figure
step(sys_exp, sys_P_cl)
figure
step(sys_P_eff)

%% PI bode controller design
close all;

% figure
% margin(sys_exp)
phase_margin = 135;
w_g = 63.4;
mag_w_g = 4.21;

PI_Kp = 10 ^ (- mag_w_g / 20);
PI_Ki = PI_Kp * w_g / 10;
sys_PI_cl = feedback((PI_Kp + PI_Ki/s) * sys_exp, 1);
sys_PI_eff = (1 - (PI_Kp + PI_Ki/s) * sys_exp) * (PI_Kp + PI_Ki/s) * 4;

% figure
% margin( (PI_Kp + PI_Ki/s) * sys_exp );

figure
step(sys_exp, sys_PI_cl);

%% PI controller root locus design
close all;

rlocus(sys_exp)
Kp = 1;

t_r = 0.005;
PMO = 20;
P_zeta = fzero(@(z) 100*exp(-pi*z/sqrt(1-z^2)) - PMO, 0.5);
P_omega_n = (1 - 0.4167 * P_zeta + 2.917 * P_zeta^2) / t_r;

figure
hold on
plot(-P_omega_n * P_zeta, P_omega_n * sqrt(1 - P_zeta^2), '*', 'MarkerSize', 24);
rlocus(sys_exp / (s * (1 + Kp * sys_exp)))
hold off

Ki = 150;

figure
step(sys_exp, feedback((Kp + Ki/s) * sys_exp, 1))


%% plot data utility function
function [] = plot_data(data, plot_title, varargin)
    if length(varargin) > 0
        sys = varargin{1};
    end
    if length(varargin) > 1
        break_freq = varargin{2};
    end
    data(:,4) = - data(:,1) .* data(:,4) * 360;
    data(:,3) = data(:,3) ./ data(:,2);
    data(:,3) = 20 * log10(data(:,3));
    data(:,1) = 2 * pi * data(:,1);
    
    if length(varargin) > 0
        [mag, phase, w] = bode(sys, {min(data(:,1)), max(data(:,1))});
        mag = squeeze(mag);
        phase = squeeze(phase);
        mag = 20 * log10(mag);
    end

    figure
    subplot(1,2,1)
    hold on
    plot(data(:, 1), data(:,3), 'o');
    if length(varargin) > 0
        plot(w, mag);
    end
    if length(varargin) > 1
        plot([break_freq * 2 * pi, break_freq * 2 * pi], [min(mag), max(mag)], 'black--');
    end
    set(gca, 'XScale', 'log');
    xlabel("Frequency [rad/s]")
    ylabel("Magnitude [dB]")
    title(plot_title)
    hold off
    
%     figure
    subplot(1,2,2)
    hold on
    plot(data(:, 1), data(:,4), 'o');
    if length(varargin) > 0
        plot(w, phase);
    end
    if length(varargin) > 1
        plot([break_freq * 2 * pi, break_freq * 2 * pi], [min(phase), max(phase)], 'black--');
    end
    set(gca, 'XScale', 'log');
    xlabel("Frequency [rad/s]")
    ylabel("Phase [degrees]")
%     title(plot_title)
    hold off
end