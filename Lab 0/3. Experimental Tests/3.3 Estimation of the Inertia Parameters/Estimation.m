%% Inertia Estimation

N_periods = 10;
PeriodLen = 2;

% Extract Simulation Data
a_m   = struct('Time', out.a_m.Time, 'Values', out.a_m.Data);
tau_i = struct('Time', out.tau_i.Time, 'Values', out.tau_i.Data);

% Calculate Averages
a_m_Mean   = WindowAverage(  a_m, PeriodLen, N_periods, [0.4, 0.9], [1.4, 1.9], 1);
Tau_i_Mean = WindowAverage(tau_i, PeriodLen, N_periods, [0.4, 0.8], [1.4, 1.8], 1);

% Computing Estimated Inertia
J_hat_Arr = (Tau_i_Mean(:, 1) - Tau_i_Mean(:, 2)) ...
         ./   (a_m_Mean(:, 1) -   a_m_Mean(:, 2));

Jeq_hat = mean(J_hat_Arr);

%% Save Estimated Parameter
save("../../utils/BBParams.mat", "Jeq_hat", "-append");