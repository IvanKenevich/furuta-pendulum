clear; clc; close all;

freqs = [0.1, 0.5, 1, 2, 4, 8];

for i = 1:length(freqs)
    fname = string(freqs(i)) + "hz_9v.txt";
    x = table2array(readtable(fname));
    subplot(2,3,i)
    plot(x)
    title(freqs(i))
end