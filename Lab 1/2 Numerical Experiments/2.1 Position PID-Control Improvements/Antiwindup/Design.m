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

% Simplified System Trans Data
SS.num = Km;
SS.den = [Tm * gbox.N, gbox.N, 0];
P = tf(SS.num, SS.den);

%% Controller Params
Alpha           = 20;             % Ti / Td  ratio
SettlingTime    = 0.15;           % Settling Time
Mp              = 0.1;            % Overshoot

%% Controller Design
[Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, SettlingTime, Mp, Alpha);

%% Anti-Windup Parameters
Tw = SettlingTime / 5;
Kw = 1 / Tw;

%% Input Reference
Ref = 360;
