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

N = size(A, 1);                % System Order

%% Controller Params
SettlingTime = 0.15;           % Settling Time
Mp           = 0.1;            % Overshoot

% Approximated Second Order Sys Params
Zeta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
Wn   = 3 / Zeta / SettlingTime;

% Controller Desired Poles
L1 = 2 * Wn * exp(1j * (-pi + pi/4));
L2 = conj(L1);
% L3 = Wn * exp(1j * (-pi + pi/6));
% L4 = conj(L3);
L5 = -4 * Wn;

Poles = [L1, L2, L5];

%% Reference Signal
Ref = 40;

% Exo-System Creation
Ar = 0;
Br = 0;
Cr = 1;
Dr = 0;

%% State Feedback Controller Design

% Create the Augmented System
Az = [Ar, C; zeros(2, 1), A];
Bz = [0; B];

% Controller Design
K = place(Az, Bz, Poles);

%% Real Derivative
Delta = 1 / sqrt(2);
Wc    = 2 * pi * 50;

RD.num = [Wc^2, 0];
RD.den = [1, 2*Delta*Wc, Wc^2];
