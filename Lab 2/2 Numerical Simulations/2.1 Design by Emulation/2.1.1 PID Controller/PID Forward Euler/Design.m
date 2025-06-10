clear
close all
clc

addpath("../../../../utils/");

%% Model Parameter Declaration
params;

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
Alpha           = 40;             % Ti / Td  ratio
SettlingTime    = 0.15;           % Settling Time
Mp              = 0.1;            % Overshoot

% Discrete Simulation Specs
Ts = 1e-3;

%% Controller Design
[Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, SettlingTime, Mp, Alpha);

% Create the Continuous PID Controller
s = tf('s');
C = Kp + Ki/s + Kd*s / (Tl*s + 1);

C_d = c2d_euler(C, Ts, 'forward');

% Controller Data
[PID.num, PID.den] = tfdata(C_d);

PID.num = cell2mat(PID.num);
PID.den = cell2mat(PID.den);

%% Input Reference
Ref = 50;
