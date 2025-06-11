clear
close all
clc

%% Parameters
A = [0, 1; 0, -20];
B = [0; 60];
C = [1, 0];
D = 0;

%% Design to Track a Sine Wave and a Step
% Input System
W = 6;
Ap = [0, 1, 0; 0, 0, 1; 0 -W^2, 0];
Cp = [1, 0, 0];

% Augmented System
Ae = [Ap, zeros(3, 2); B * Cp, A];
Be = [zeros(3, 1); B];
Ce = [zeros(1, 3), C];

% Pole Placement
SettlingTime = 0.15;           % Settling Time
Mp           = 0.1;            % Overshoot

% Approximated Second Order Sys Params
Zeta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
Wn   = 3 / Zeta / SettlingTime;

% Desired Poles
Lc1 = -Zeta * Wn + 1j * Wn * sqrt(1 - Zeta^2);
Lc  = [Lc1, conj(Lc1)];

K   = place(A, B, Lc);

%% Observer Design
Le1 =  5 * Wn * exp(1j*(-pi + pi/3));
Le2 = conj(Le1);
Le3 =  5 * Wn * exp(1j*(-pi + pi/6));
Le4 = conj(Le3);
Le5 = -5 * Wn;

Le = [Le1, Le2, Le3, Le4, Le5];
L = place(Ae', Ce', Le)'; 

Ao = Ae - L*Ce;
Bo = [Be, L];
Co = eye(5);
Do = zeros(5, 2);

%% Discrete Observer
Ts = 0.001;

% % Forward Euler
% Ao_d = eye(5) + Ao * Ts;
% Bo_d = Bo * Ts;
% Co_d = Co;
% Do_d = Do;

% % Backward Euler
% Ao_d = inv(eye(5) - Ao * Ts);
% Bo_d = inv(eye(5) - Ao * Ts) * Bo * Ts;
% Co_d = Co * inv(eye(5) - Ao * Ts);
% Do_d = Do + Co * inv(eye(5) - Ao * Ts) * Bo * Ts;

% % Tustin
% Ao_d = (eye(5) + Ao * Ts / 2) * (eye(5) - Ao * Ts / 2)^-1;
% Bo_d = (eye(5) - Ao * Ts / 2)^-1 * Bo * sqrt(Ts);
% Co_d = sqrt(Ts) * Co * (eye(5) - Ao * Ts / 2)^-1;
% Do_d = Do + Co * (eye(5) - Ao * Ts / 2)^-1 * Bo * Ts / 2;

% Exact
P_d = c2d(ss(Ao, Bo, Co, Do), Ts, 'zoh');
[Ao_d, Bo_d, Co_d, Do_d] = ssdata(P_d);
