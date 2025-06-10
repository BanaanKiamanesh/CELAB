clear
close all
clc

%% Init
s = tf('s');
P = 90 / (s^2 + 60*s + 3);
P = ss(P);

% Desired Response Characteristics
RiseTime = 0.25;            % Rise Time
Mp = 0.2;                   % Overshoot Percent

% Discretize System
Ts = RiseTime / 30;
P_d = c2d(P, Ts, 'zoh');

% Continuous Time System Property Matrices
A = P.A;
B = P.B;
C = P.C;
D = P.D;

% Discrete Time System Property Matrices
Ad = P_d.A;
Bd = P_d.B;
Cd = P_d.C;
Dd = P_d.D;

%% Controller Design
% Compute the Equivalent Second Order System Properties
Delta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);
Wn    = 1.8 / RiseTime; 

% Compute the Poles
CtrlPoles = roots([1, 2*Delta*Wn, Wn^2]);
CtrlPolesZ = exp(Ts * CtrlPoles);                % Send Poles to z Domain

% Pole Placement
K = place(Ad, Bd, CtrlPolesZ);

% Gain Calculation
tmp = [Ad - eye(2), Bd; Cd, 0] \ [0, 0, 1]';
Nx = tmp(1:2);
Nu = tmp(end);

Nr = Nu + K*Nx;

%% Observer Design
% Observer Poles
ObsvPoles = CtrlPoles * 5;
ObsvPolesZ = exp(Ts * ObsvPoles);                % Send Poles to z Domain

% Observer Gain Matrix
L = place(Ad', Cd', ObsvPolesZ)';

% Observer
Ao = Ad - L*Cd;
Bo = [Bd, L];
Co = eye(2);
Do = zeros(2);





