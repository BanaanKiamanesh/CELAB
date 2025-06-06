clear
close all
clc

addpath("../../utils/");

%% Model Parameter Declaration
params;
load BBParams.mat

%% Simplified Plant Model
Req = mot.R + sens.curr.Rs;
Jeq = mot.J + mld.JD/gbox.N^2;

Km = drv.dcgain*mot.Kt / (Req * Beq + mot.Kt*mot.Ke);
Tm = Req * Jeq / (Req * Beq + mot.Kt*mot.Ke);

% Simplified System Trans Data
SS.num = Km;
SS.den = [Tm * gbox.N, gbox.N, 0];
P = tf(SS.num, SS.den);

%% Controller Params
Alpha           = 20;              % Ti / Td  ratio
SettlingTime    = 0.15;           % Settling Time
Mp              = 0.1;            % Overshoot

%% Controller Design
[Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, SettlingTime, Mp, Alpha);

%% Input Reference
Ref = 50;

%% Real Derivative High-Pass + Low-Pass Filter Params
Wc = 2 * pi * 20;
Delta = 1/sqrt(2);

RD.num = [Wc^2, 0];
RD.den = [1, 2*Wc*Delta, Wc^2];

LP.num = Wc^2;
LP.den = [1, 2*Wc*Delta, Wc^2];
