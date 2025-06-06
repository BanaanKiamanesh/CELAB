%% Friction Estimation
%>>>>>>>>>>>>> Positive Velocity Interval
t = out.tout;
IdxPos = t <= 45;

W_m_Pos        = struct("Time", double(out.W_m.Time), "Values", double(out.W_m.Data));
W_m_Pos.Time   = W_m_Pos.Time(IdxPos);
W_m_Pos.Values = W_m_Pos.Values(IdxPos); 

Tau_m_Pos        = struct("Time", double(out.Tau_m.Time), "Values", double(out.Tau_m.Data));
Tau_m_Pos.Time   = Tau_m_Pos.Time(IdxPos);
Tau_m_Pos.Values = Tau_m_Pos.Values(IdxPos); 

% Get the Mean
W_m_Mean   = WindowAverage(  W_m_Pos, 5, 9, [1, 4], [], 1);
Tau_m_Mean = WindowAverage(Tau_m_Pos, 5, 9, [1, 4], [], 1);

Phi_T = [W_m_Mean, (1/gbox.N)*sign(W_m_Mean)];

Y           = Tau_m_Mean;
thLS_hat    = Phi_T \ Y;
Beq_hat_Pos = thLS_hat(1);
Tau_sf_Pos  = thLS_hat(2);

%>>>>>>>>>>>>> Negative Velocity Interval
IdxNeg = t > 45;

W_m_Neg        = struct("Time", double(out.W_m.Time), "Values", double(out.W_m.Data));
W_m_Neg.Time   = W_m_Neg.Time(IdxNeg) - 45;
W_m_Neg.Values = W_m_Neg.Values(IdxNeg); 

Tau_m_Neg        = struct("Time", double(out.Tau_m.Time), "Values", double(out.Tau_m.Data));
Tau_m_Neg.Time   = Tau_m_Neg.Time(IdxNeg) - 45;
Tau_m_Neg.Values = Tau_m_Neg.Values(IdxNeg); 

% Get the Mean
W_m_Mean   = WindowAverage(  W_m_Neg, 5, 9, [1, 4], [], 1);
Tau_m_Mean = WindowAverage(Tau_m_Neg, 5, 9, [1, 4], [], 1);

Phi_T = [W_m_Mean, (1/gbox.N)*sign(W_m_Mean)];

Y           = Tau_m_Mean;
thLS_hat    = Phi_T \ Y;
Beq_hat_Neg = thLS_hat(1);
Tau_sf_Neg  = thLS_hat(2);

% Get the Final Estimated Values
Beq    = (Beq_hat_Pos + Beq_hat_Neg) / 2;
Tau_sf = (abs(Tau_sf_Pos) + abs(Tau_sf_Neg)) / 2;

%% Save Data
save('../../utils/BBParams.mat', "Beq", "Tau_sf");