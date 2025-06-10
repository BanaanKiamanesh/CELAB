clear
close all
clc

addpath("../../../utils/");

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

%% State Feedback Controller Design
K = place(A, B, Poles);

% Computing FeedForward Gains
tmp = [A, B; C, 0] \ [0; 0; 1];
Nx = tmp(1:N, 1);
Nu = tmp(end);

Nr = Nu + K * Nx;

%% Reference Signal
Ref = 70;

%% Real Derivative
Delta = 1 / sqrt(2);
Wc    = 2 * pi * 50;

RD.num = [Wc^2, 0];
RD.den = [1, 2*Delta*Wc, Wc^2];
