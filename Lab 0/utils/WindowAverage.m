function AvgVal = WindowAverage(Sig, T, N_periods, Win1, Win2, ShowPlot)

    %WindowAverage Function to Define Two Temporal Windows in Which Compute the Average
    %   of the Values of a Signal in Each Period.
    %
    %   Input:  - Sig: Simulink Struct with Time from Workspace in the Form out.[name]
    %           - T: Period of the Signal [s]
    %           - N_periods: Number of Periods of the Signal
    %           - Win1: First Averaging Window
    %           - Win2: Second Averaging Window
    %           - ShowPlot: '1' to Plot the Windowed Signal, '0' Otherwise
    %
    %   Output: - AvgVal: Vector Containing the Averages in Each Window of Each Period

    if ~isempty(Win2) % If Only a Single Averaging Window is Passed

        % Array to Store Average Values for Each Period
        AvgVal = zeros(N_periods, 2);
        Val    = Sig.Values;
        t      = Sig.Time;
        for k = 1:N_periods
            % Define Time Limits for this Period
            t_start = (k-1) * T;

            % Extract Indices for the Two Windows in This Period
            Idx1 = (t >= t_start + Win1(1)) & (t < t_start + Win1(2));
            Idx2 = (t >= t_start + Win2(1)) & (t < t_start + Win2(2));

            % Compute the Mean in Each Window
            AvgVal(k, 1) = mean(Val(Idx1));
            AvgVal(k, 2) = mean(Val(Idx2));
        end

        % Display Results
        disp('Averages for each period:');
        disp(array2table(AvgVal, 'VariableNames', {'Window1', 'Window2'}));

        if ShowPlot
            % Plot Signal with Highlighted Windows
            figure;
            plot(t, Val, 'k');
            hold on;
            for k = 1:N_periods
                t_start = (k-1) * T;
                fill([t_start + Win1(1), t_start + Win1(2), t_start + Win1(2), t_start + Win1(1)], ...
                    [min(Val), min(Val), max(Val), max(Val)], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
                fill([t_start + Win2(1), t_start + Win2(2), t_start + Win2(2), t_start + Win2(1)], ...
                    [min(Val), min(Val), max(Val), max(Val)], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            end

            xlabel('Time (s)');
            ylabel('Signal Value');
            title('Averaging Windows');
            legend({'Signal', 'Window 1', 'Window 2'});
            grid on;
        end

    else
        % Array to Store Average Values for Each Period
        AvgVal = zeros(N_periods, 1);
        Val    = Sig.Values;
        t      = Sig.Time;
        for k = 1:N_periods
            % Define Time Limits for this Period
            t_start = (k-1) * T;

            % Extract Indices for the Two Windows in This Period
            Idx1 = (t >= t_start + Win1(1)) & (t < t_start + Win1(2));

            % Compute the Mean in Each Window
            AvgVal(k, 1) = mean(Val(Idx1));
        end

        % Display Results
        disp('Averages for each period:');
        disp(array2table(AvgVal, 'VariableNames', {'Window'}));

        if ShowPlot
            % Plot Signal with Highlighted Windows
            figure;
            plot(t, Val, 'k');
            hold on;
            for k = 1:N_periods
                t_start = (k-1) * T;
                fill([t_start + Win1(1), t_start + Win1(2), t_start + Win1(2), t_start + Win1(1)], ...
                    [min(Val), min(Val), max(Val), max(Val)], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            end
            xlabel('Time (s)');
            ylabel('Signal Value');
            title('Averaging Window');
            legend({'Signal', 'Window'});
            grid on;
        end
    end
end
