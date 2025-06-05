clear
close all
clc

%% Model Parameter Declaration
run("../../utils/params.m");

%% Simplified Plant Model
Req = mot.R + sens.curr.Rs;
Beq = 0;
Jeq = mot.J + mld.JD/gbox.N^2;

Km = drv.dcgain*mot.Kt / (Req * Beq + mot.Kt*mot.Ke);
Tm = Req * Jeq / (Req * Beq + mot.Kt*mot.Ke);

% Simplified System Trans Data
SS.num = Km;
SS.den = [Tm * gbox.N, gbox.N, 0];
P = tf(SS.num, SS.den);

%% Controller Params
Alpha = 4;              % Ti / Td  ratio
Ts    = 0.15;           % Settling Time
Mp    = 0.1;            % Overshoot

%% Controller Design
[Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, Ts, Mp, Alpha);

%% Input Reference
Ref = 50;
