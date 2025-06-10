clear
close all
clc

addpath("../../../utils/");

%% Model Parameter Declaration
params;
load BBParams.mat

%% Simplified Plant Model
Req = mot.R + sens.curr.Rs;
% Jeq = mot.J + mld.JD/gbox.N^2;
Jeq = Jeq_hat;

Km = drv.dcgain*mot.Kt / (Req * Beq + mot.Kt*mot.Ke);
Tm = Req * Jeq / (Req * Beq + mot.Kt*mot.Ke);

% Simplified System Trans Data
SS.num = Km;
SS.den = [Tm * gbox.N, gbox.N, 0];
P = tf(SS.num, SS.den);

%% Controller Params
Alpha           = 20;             % Ti / Td  ratio
SettlingTime    = 0.15;           % Settling Time
Mp              = 0.1;            % Overshoot

%% PID Controller Design
[Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, SettlingTime, Mp, Alpha);

%% FeedForward Compensation Parameters
% Inertia Compensation
I_C = (gbox.N * Req * Jeq_hat)/(drv.dcgain * mot.Kt);

% Friction Compensation
F_C = Req/(drv.dcgain * mot.Kt * gbox.N);

% BEMF Compensation
B_C = (gbox.N * mot.Ke)/drv.dcgain;

%% Reference Signal

% Periodic Acceleration Reference
a_l_ref = [900, 0, -900, -900, 0, 900];
