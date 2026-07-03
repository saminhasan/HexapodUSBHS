clear; clc; close all;

% Settings
fs = 1000;      % Hz
fc = 20;        % Hz

inFile  = "SynthTraj.csv";
outFile = "filtered_ SynthTraj.csv";

% Load CSV: expected shape N x 6
x = readmatrix(inFile);

% Zero-phase low-pass filter
y = LPFilterZeroPhase(x, fs, fc);

% Save filtered CSV
writematrix(y, outFile);

% Time vector
t = (0:size(x,1)-1) / fs;

% Plot each column in its own figure
for k = 1:6
    figure(k);
    plot(t, x(:,k), 'DisplayName', 'Raw'); 
    hold on;
    plot(t, y(:,k), 'LineWidth', 1.5, 'DisplayName', 'Filtered');
    hold off;

    grid on;
    xlabel('Time [s]');
    ylabel(sprintf('Column %d', k));
    title(sprintf('Zero-phase LP filtered signal - Column %d', k));
    legend;
end

disp("Filtered CSV saved to: " + outFile);


function filteredSignal = LPFilterZeroPhase(signal, fs, fc)
    w1 = 2*pi*fc;
    zeta1 = 0.707;

    % Analog 3rd-order LP:
    % H(s) = w1^3 / ((s + w1)(s^2 + 2*zeta*w1*s + w1^2))
    Bw = w1^3;
    Aw = conv([1, w1], [1, 2*zeta1*w1, w1^2]);

    % Discretize
    Gz = c2d(tf(Bw, Aw), 1/fs, "tustin");

    Bz = Gz.Numerator{1};
    Az = Gz.Denominator{1};

    % Zero-phase filtering, column-wise
    filteredSignal = filtfilt(Bz, Az, signal);
end