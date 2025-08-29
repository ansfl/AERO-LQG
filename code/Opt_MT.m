%% LQG Simulation with Random Initialization
% This example illustrates system behavior when Q and R are chosen
% arbitrarily, without guidance from Bryson's rule or other tuning
% methods. Random weights often lead to instability or divergence,
% highlighting the importance of systematic optimization.

% Bryson's Rule Initialization
[Q_Bry, R_Bry] = S_Bryson();     % Baseline LQG control weights

% Random Q,R Initialization
Q_rnd = tril(randn(size(Q_Bry)));   % random lower-triangular
R_rnd = randn(size(R_Bry));         % random matrix
[X_GT, X_est, u_out_k] = run_LQG_simulation(Q_rnd*Q_rnd', R_rnd*R_rnd');

% Visualization
visualize_x;      % State trajectories
visualize_err;    % Error states and control inputs