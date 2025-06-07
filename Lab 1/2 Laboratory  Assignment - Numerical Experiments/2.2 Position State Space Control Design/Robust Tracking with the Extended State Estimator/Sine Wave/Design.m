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

% Desired Poles
Lc1 = -Zeta * Wn + 1j * Wn * sqrt(1 - Zeta^2);
CtrlPoles = [Lc1, conj(Lc1)];

Le1 =  2 * Wn * exp(1j*(-pi + pi/3));
Le2 = conj(Le1);
Le3 =  2 * Wn * exp(1j*(-pi + pi/6));
Le4 = conj(Le3);
Le5 = -2 * Wn;

ObsvPoles = [Le1, Le2, Le3, Le4, Le5];

%% Reference Signal
% Sine Wave Reference
Tr = 0.5;
Amp = 40;

% Step Disturbance
Cw = 1;

% Exo System
W0 = 2*pi / Tr;

Ar = [0, 1, 0; 0, 0, 1; 0, -W0^2, 0];
Br = zeros(3, 1);
Cr = [1, 0, 0];
Dr = 0;

%% State Feedback Controller Design
% Extended Plant Model
Ae = [Ar, zeros(3, 2); B*Cr, A];
Be = [zeros(3, 1); B];
Ce = [zeros(1, 3), C];

% Controller Design
K = place(  A,   B, CtrlPoles);
L = place(Ae', Ce', ObsvPoles)';















