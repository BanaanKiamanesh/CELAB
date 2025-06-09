clear
close all
clc

addpath("../../../utils/");

%% Model Parameter Declaration
params;

%% Simplified Plant Model
Req  = mot.R + sens.curr.Rs;
Beq  = mot.B + mld.Bh/gbox.N^2;
Jeq  = mot.J + mld.Jh/gbox.N^2;
Bb   = mld.Bb;
Jb   = mld.Jb;
k    = mld.k;
Kt   = mot.Kt;
Ke   = mot.Ke;
N    = gbox.N;
kdrv = drv.dcgain;
Tdrv = drv.Tc;

% State Space Model of the System
A = zeros(4);
A(1:2, 3:4) = eye(2);
A(3:4, 1:2) = [k/N^2/Jeq * [-1, 1]; [1, -1] * k/Jb];
A(3:4, 3:4) = diag([-1/Jeq * (Beq + Kt*Ke/Req), -Bb/Jb]);

B = zeros(4, 1);
B(3) = Kt * kdrv / N / Jeq / Req;

Bd = zeros(4, 1);
Bd(3) = -1 / N^2 / Jeq;

% Change of Basis Matrix
T = eye(4);
T(2, 1) = 1;
T(4, 3) = 1;

% Change of Basis
A  = T \ A * T;
B  = T \ B;
Bd = T \ Bd;
C  = [1, 0, 0, 0];
D  = 0;

N = size(A, 1);             % System Order

%% Controller Params
SettlingTime    = 0.85;           % Settling Time
Mp              = 0.3;            % Overshoot

% Approximated Second Order Sys Params
Zeta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
Wn   = 3 / Zeta / SettlingTime;
Phi  = atan2(sqrt(1 - Zeta^2), Zeta); 

% Controller Desired Poles
L1 = Wn * exp(1j * (-pi + Phi));
L2 = conj(L1);
L3 = Wn * exp(1j * (-pi + Phi/2));
L4 = conj(L3);
L5 = -Wn;

Poles = [L1, L2, L3, L4, L5];

%% State Feedback Controller Design

% Augmented System Matrices
Ae = [0, C; zeros(4, 1), A];
Be = [0; B];
Ce = [0, C];

TmpK = place(Ae, Be, Poles);

Ki = TmpK(1);
K = TmpK(2:end);

% Computing FeedForward Gains
tmp = [A, B; C, 0] \ [0; 0; 0; 0; 1];
Nx = tmp(1:N, 1);
Nu = tmp(end);

Nr = Nu + K * Nx;

%% Reference Signal
Ref = 50;

%% Real Derivative
Delta = 1 / sqrt(2);
Wc    = 2 * pi * 50;

RD.num = [Wc^2, 0];
RD.den = [1, 2*Delta*Wc, Wc^2];

%% Observer Design for Simple SS System
OsbvPoles = Poles(1:4) * 5;

L = place(A', C', OsbvPoles)';

% Luenberger Observer
Ao = A - L*C;
Bo = [B, L];
Co = C;
Do = [D, 0];
