clear
close all
clc

addpath("../../utils/");

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

km = (kdrv * Kt) / (Req * Beq + Kt * Ke);
Tm = (Req * Jeq) / (Req * Beq + Kt * Ke);

% Transfer Function Creation
s = tf('s');
D_tau_prime = Jeq * Jb * s^3 ...
            + (Jeq * Bb + Jb * Beq) * s^2 ...
            + (Beq * Bb + k * (Jeq + (Jb / N^2))) * s ...
            + k * (Beq + (Bb / N^2));

P = (1 / (N * s)) * (kdrv * Kt * (Jb * s^2 + Bb * s + k)) ...
    / ((Req * D_tau_prime) ...
    + Kt * Ke * (Jb * s^2 + Bb * s + k));

%% Controller Params
Alpha           = 6;             % Ti / Td  ratio
SettlingTime    = 0.7;           % Settling Time
Mp              = 0.3;           % Overshoot

%% Controller Design
[Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, SettlingTime, Mp, Alpha);

% Anti-windup Parameter
Tw = SettlingTime / 5;
Kw = 1 / Tw;

%% Input Reference
Ref = 50;
