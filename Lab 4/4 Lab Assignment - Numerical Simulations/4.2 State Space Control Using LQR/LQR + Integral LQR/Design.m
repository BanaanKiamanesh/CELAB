clear
close all
clc

addpath("../../../utils/");

%% Load Model Params
params;

%% Create System State Space Matrices
% System M Matrix
M12 = 2*gbox.N * (1-gbox.N) * mot.rot.Iyy ...
    + (body.m * body.zb + 2 * mot.rot.m * mot.rot.zb) * wheel.r;

M11 = 2 * wheel.Iyy ...
    + 2 * gbox.N^2 * mot.rot.Iyy ...
    + (body.m + 2 * wheel.m + 2 * mot.rot.m) * wheel.r^2;

M22 = body.Iyy ...
    + 2 * ((1-gbox.N)^(2)) * mot.rot.Iyy ...
    + body.m * body.zb^2 ...
    + 2 * mot.rot.m * mot.rot.zb^2;

M21 = M12;

M = [M11, M12; M21, M22];

% C Matrix
C       = zeros(2, 2);
C(1, 2) = -(body.m * body.zb + 2 * mot.rot.m * mot.rot.zb) * wheel.r;

% Gravity Vector
g_q    = zeros(2, 1);
g_q(2) = -g * (body.m * body.zb + 2 * mot.rot.m * mot.rot.zb);

% G
G = [0, 0; 0, g_q(2)];

% Create Fv1
b   = gbox.N^2 * mot.B + gbox.B;
F11 = 2 * (gbox.B + wheel.B);
F12 = -2 * gbox.B;
F21 = F12;
F22 = 2 * gbox.B;

Fv1 = [F11, F12; F21, F22];

Fv1_d = Fv1 + (2 * gbox.N^(2) * mot.Kt * mot.Ke / mot.R) * [1, -1; -1, 1];

% State Space Model
A = [0, 0, 1, 0
     0, 0, 0, 1
     -M \ G, -M \ Fv1_d];
B = 2 * gbox.N * mot.Kt / mot.R * [0, 0; 0, 0; inv(M)] * [1; -1];
C = [1, 0, 0, 0];
D = 0;

P = ss(A, B, C, D);

% Discretize the System
Pd = c2d(P, Ts, 'zoh');
Ad = Pd.A;
Bd = Pd.B;
Cd = Pd.C;
Dd = Pd.D;

%% LQR Controller Design
ro        = 100;
Gamma_bar = pi/18;
Theta_bar = pi/360;
U_bar     = 1;

Q = diag([1/(Gamma_bar)^2, 1/(Theta_bar^2), 0, 0]);
r = 1 / U_bar^2;
[K, S, e] = dlqr(Ad, Bd, Q, r*ro);

tmp = [Ad - eye(4), Bd; C, 0] \ [0; 0; 0; 0; 1];
Nx = tmp(1:end-1);
Nu = tmp(end);

Nr = Nu + K * Nx;

%% Integral LQR Design
q11 = 0.2;
Q   = diag([q11, 1/(Gamma_bar)^2, 1/(Theta_bar^2), 0, 0]);

Phi_I   = [1, Cd; zeros(4,1), Ad];
Gamma_I = [0; Bd];

K_I = dlqr(Phi_I, Gamma_I, Q, r*ro);
Ki  = K_I(1);
K_I = K_I(2:end);

tmp  = [Ad - eye(4), Bd; Cd, 0] \ [0; 0; 0; 0; 1];
Nx_I = tmp(1:4);
Nu_I = tmp(end);

Nr_I = Nu_I + K_I * Nx_I;

%% Filters
% Complementary Filter
fc = 0.35;
Tc = 1/(fc*2*pi);

c = Ts / (Tc+Ts);
CF.num = c;
CF.den = [1, -(1-c)];

% Discrete Real Derivative
RD.den = [3*Ts, 0, 0, 0];
RD.num = [1, 0, 0, -1];