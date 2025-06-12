clear
close all
clc

%% Init
A = [0, 1; 0, -10];
B = [0; 120];
C = [1, 0];
D = 0;

s = tf('s');
P = ss(A, B, C, D);

%% PID Design
% Controller Properties
RiseTime = 0.15;        % Rise Time
Mp = 0.2;               % Overshoot Percent
Alpha = 500;            % Ti / Td

% PID for Step Track
[KpStep, KiStep, KdStep, TlStep] = DesignPID(P, RiseTime, Mp, Alpha);

% PID for Ramp Track
% (It has a Pole in the Origin So Same Thing, Else Design for P(s)/s)
[KpRamp, KiRamp, KdRamp, TlRamp] = DesignPID(P, RiseTime, Mp, Alpha);

% Anti-Windup
Kw = 1 / (RiseTime / 5);

%% Aux Functions
function [Kp, Ki, Kd, Tl] = DesignPID(P, RiseTime, Mp, Alpha)

    % Step 1: Damping Ratio
    Delta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);

    % Step 2: Cross Over Freq
    Wgc = 1.8 / RiseTime;

    % Step 3: Phase Margin
    Phi = atan2(2*Delta, sqrt(sqrt(1 + 4*Delta^4) - 2*Delta^2));

    % Step 4: |P(j*Wgc)|
    PWgc = evalfr(P, 1j * Wgc);

    % Step 5: Gain and Phase Compensation
    PMag = abs(PWgc);
    DeltaK = 1 / PMag;
    DeltaPhi = -pi + Phi - angle(PWgc);

    % Step 6: PID Params
    Kp = DeltaK * cos(DeltaPhi);

    Td = (tan(DeltaPhi) + sqrt(tan(DeltaPhi)^2 + 4 / Alpha)) / (2 * Wgc);
    Ti = Alpha * Td;

    Kd = Kp * Td;
    Ki = Kp / Ti;

    % Real Derivative (To Make it Physically Implementable)
    Tl = 1 / 5 / Wgc;
end