clear
close all
clc

addpath("../../../../utils/");

%% Model Parameter Declaration
params;
load BBParams.mat

%% Simplified Plant Model
Req = mot.R + sens.curr.Rs;
Jeq = Jeq_hat;

Km = drv.dcgain*mot.Kt / (Req * Beq + mot.Kt*mot.Ke);
Tm = Req * Jeq / (Req * Beq + mot.Kt*mot.Ke);

% Simplified System State Space Model
A = [0, 1; 0, -1/Tm];
B = [0; Km/Tm/gbox.N];
C = [1, 0];
D = 0;

N = size(A, 1);             % System Order

%% Augmented System
Ae = [0, C; zeros(2, 1), A];
Be = [0; B];
Ce = [0, C];

%% Controller Params
SettlingTime    = 0.15;           % Settling Time
Mp              = 0.1;            % Overshoot

% Approximated Second Order Sys Params
Zeta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
Wn   = 3 / Zeta / SettlingTime;

% Controller Desired Poles
L1 = -Zeta * Wn + 1j * Wn * sqrt(1-Zeta^2);
L2 = conj(L1);
L3 = -Zeta * Wn;

Poles = [L1, L2, L3];

% Observer Pole
ObsvPole = real(L1) * 5;

% Sampling Time
Ts = 1e-3;

%% Reduced Order State Observer Design
L = (A(2, 2) - ObsvPole) / A(1, 2);

% Ao = ObsvPole;
Ao = A(2, 2) - L * A(1, 2);
Bo = [B(2) - L*B(1), Ao * L + A(2, 1) - L * A(1, 1)];
Co = [0; 1];
Do = [0, 1; 0, L];

%% Discretizing the Observer
% Forward Euler Method
Po = 1 + Ao * Ts;
Go = Bo * Ts;
Ho = Co;
Jo = Do;

%% State Feedback Controller Design
K = place(Ae, Be, Poles);

Ki = K(1);
K  = K(2:end);

% Computing FeedForward Gains
tmp = [A, B; C, 0] \ [0; 0; 1];
Nx = tmp(1:N, 1);
Nu = tmp(end);

Nr = Nu + K * Nx;

%% Reference Signal
Ref = 70;
