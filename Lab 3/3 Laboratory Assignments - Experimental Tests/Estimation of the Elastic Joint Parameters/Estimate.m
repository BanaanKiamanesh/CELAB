%% Get the Data Ready
Sig = out.Data.signals.values;
t = out.Data.time;

%% Find Peaks
[Peaks, PeakTime] = findpeaks(abs(Sig), t, 'MinPeakDistance', 0.1);

%% Parameter Estimation
M = numel(Peaks);           % Number of Detected Peak Points

% Solve Regression Problem
Y            = log(Peaks);
Phi          = [-(0:M-1)', ones(M,1)];
Theta_hat_Ls = Phi \ Y;
Espilon_hat  = Theta_hat_Ls(1); 

% Damping factor estimation
Delta_hat = Espilon_hat / sqrt(pi^2 + Espilon_hat^2);

% Natural Frequency Estimation
Tk = diff(PeakTime);
Omega_k_hat = pi ./ Tk;
Omega_hat = mean(Omega_k_hat);
Omega_n_hat = Omega_hat/sqrt(1-Delta_hat^2);


% Nominal value for beam moment of inertia Jb
Jb = 1.4e-3; % [kg*m^2]

% Beam viscous friction coefficient estimation
Bb_hat = Jb * 2 * Delta_hat * Omega_n_hat;

% Joint stiffness estimation
k_hat = Jb * Omega_n_hat^2;

%% Plotting
figure("Name", "Results", "Units", "normalized", "OuterPosition", [0.1, 0.1, 0.8, 0.8]);

% Noisy Signal with Smoothed Signal
subplot(3, 1, 1);
plot(t, Sig, 'b', 'LineWidth', 1.2); 

xlabel('Time [s]');
ylabel('\theta_d [deg]');
title('Original and Smoothed Beam Displacement');
grid on;

subplot(3, 1, 2);
plot(t, abs(Sig), 'b', 'LineWidth', 1.2); 
hold on;
plot(PeakTime, abs(Peaks), 'r-o', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

xlabel('Time [s]');
ylabel('|\theta_d| [deg]');
title('Peaks');
grid on;

% Noisy Signal with Estimated Signal
subplot(3, 1, 3);
plot(t, Sig, 'b', 'LineWidth', 1.2); 
hold on;

% Define the Estimated Signal Using the Calculated Parameters
A_est = Peaks(1);
theta_d_est = A_est * exp(-Delta_hat * Omega_n_hat * t) .* ...
    cos(Omega_n_hat * sqrt(1 - Delta_hat^2) * t);

plot(t, theta_d_est, 'r--', 'LineWidth', 1.2);

xlabel('Time [s]');
ylabel('\theta_d [deg]');
title('Original and Estimated Beam Displacement');
legend('Original Signal', 'Estimated Signal');
grid on;

%% Save Estimated Values
save("BBParams.mat", "k_hat", "Bb_hat");