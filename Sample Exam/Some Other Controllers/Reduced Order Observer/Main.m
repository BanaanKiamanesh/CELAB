clear
close all
clc


%% Init
A = [0, 1; 0, -36];
B = [0; 180];
C = [1, 0];
D = 0;

N = size(A, 1);             % System Order

%% Controller Params
SettlingTime    = 0.15;           % Settling Time
Mp              = 0.1;            % Overshoot

% Approximated Second Order Sys Params
Zeta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
Wn   = 3 / Zeta / SettlingTime;

% Controller Desired Poles
L1 = -Zeta * Wn + 1j * Wn * sqrt(1-Zeta^2);
L2 = conj(L1);

Poles = [L1, L2];

% Observer Pole
ObsvPole = real(L1) * 5;

% Sampling Time
Ts = 1e-3;

% Discretize the System
P = ss(A, B, C, D);
P_d = c2d(P, Ts, 'zoh');
[Phi, Gamma, H, J] = ssdata(P_d);

% Mapping Poles to Z Domain
Poles    = exp(Poles * Ts);
ObsvPole = exp(ObsvPole * Ts);

%% Discrete Time Reduced Order State Observer Design
L = (Phi(2, 2) - ObsvPole) / Phi(1, 2);

% Po = ObsvPole;
Po = Phi(2, 2) - L * Phi(1, 2);
Go = [Gamma(2) - L*Gamma(1), Po * L + Phi(2, 1) - L * Phi(1, 1)];
Ho = [0; 1];
Jo = [0, 1; 0, L];

%% State Feedback Controller Design
K = place(Phi, Gamma, Poles);

% Computing FeedForward Gains
tmp = [Phi - eye(2), Gamma; H, 0] \ [0; 0; 1];
Nx = tmp(1:N, 1);
Nu = tmp(end);

Nr = Nu + K * Nx;
