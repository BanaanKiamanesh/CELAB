function [Kp, Ki, Kd, Tl, Wgc] = DesignPIDBode(P, Ts, Mp, Alpha)
    % DesignPIDBode - PID Controller Design via Frequency-Domain Bode Method
    %
    % Inputs:
    %   P      - Transfer Function of the Plant
    %   ts     - Desired Settling Time (s)
    %   mp     - Maximum Overshoot (e.g., 0.1 for 10%)
    %   alpha  - Ratio Ti/Td
    %   tl     - Constant for Derivative Filter
    %
    % Outputs:
    %   Kp, Ki, Kd - PID Controller Gains
    %   wgc    - Gain Crossover Frequency (rad/s)

    % Step 1: Calculate Damping Ratio from Overshoot
    delta = log(1/Mp) / sqrt(pi^2 + log(1/Mp)^2);

    % Step 2: Desired Gain Crossover Frequency (rad/s)
    Wgc = 3 / (delta * Ts);

    % Step 3: Desired Phase Margin (phim)
    Phim = atan2(2 * delta, sqrt(sqrt(1 + 4*delta^4) - 2*delta^2));

    % Step 4: Compute Plant Transfer Function at wgc
    Pwgc = evalfr(P, 1i * Wgc);

    % Step 5: Gain and Phase Compensation
    abs_P = abs(Pwgc);
    deltaK = 1 / abs_P;
    deltaPhi = -pi + Phim - angle(Pwgc);

    % Step 6: PID Parameters from Compensation
    Kp = deltaK * cos(deltaPhi);

    Td = (tan(deltaPhi) + sqrt(tan(deltaPhi)^2 + 4 / Alpha)) / (2 * Wgc);
    Ti = Alpha * Td;

    Kd = Kp * Td;
    Ki = Kp / Ti;

    % Real Derivative Time Constant (must be between 2-5 Chosen 2 as Said in the Assignment File)
    Tl = 1 / (2*Wgc);
end
