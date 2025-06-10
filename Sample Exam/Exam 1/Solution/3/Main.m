clear
close all
clc

%% Init
A = [0, 1; -2 -40];
B = [0; 40];
C = [1, 0];
D = 0;

% Discretize System
P = ss(A, B, C, D);
Ts = 0.01;
P_d = c2d(P, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(P_d);

%% Controller Design
% Augmented System
Ae = [1, Cd; zeros(2, 1), Ad];
Be = [0; Bd];

% LQR Placement
Q = eye(3)/10; 
R = 1/20;
K = dlqr(Ae, Be, Q, R);

Ki = K(1);
K  = K(2:3);

% Feedforward Gain
tmp = [Ad - eye(2), Bd; Cd, 0] \ [0; 0; 1];
Nx  = tmp(1:2);
Nu  = tmp(3);

Nr = Nu + K * Nx;

%% Observer Design
ObsvPoles = exp(Ts * eig(A) * 5);
L = acker(Ad', Cd', ObsvPoles)';

Ao = Ad - L * Cd;
Bo = [Bd, L];
Co = eye(2);
Do = zeros(2);
