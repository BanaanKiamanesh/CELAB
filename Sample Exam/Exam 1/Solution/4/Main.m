clear
close all
clc

%% Parameters
A = [0, 1; 0, -20];
B = [0; 60];
C = [1, 0];
D = 0;

%% Controller Design
% Controller Design Specs
% Poles Must Be Smaller the -3/Ts = -15
SettlingTime = 0.2;

% Augmented Matrices
W0 = 6;
Ae = [[0, 1, 0; 0, 0, 1; 0, -W0^2, 0], ...
      [zeros(2, 2); C]; zeros(2, 3), A];
Be = [0; 0; 0; B];

% Pole Placement
CtrlPoles = [-20+4j, -20-4j, -20+2j, -20-2j, -20];
K         = place(Ae, Be, CtrlPoles);

% H Compensator
H.num = K(3:-1:1);
H.den = [1, 0, W0^2, 0];

K = K(4:5);

%% Derivative Filter
Wc    = 2*pi*40;
Delta = 1/sqrt(2);

RD.num = [Wc^2, 0];
RD.den = [1, 2*Delta*Wc, Wc^2];