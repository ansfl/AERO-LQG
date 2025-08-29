%% LQG Baseline Setup and Simulation
% This script demonstrates how Bryson's rule provides a stable baseline
% for LQG control weight selection. It also includes an optional example
% of divergence when Q and R are chosen randomly, highlighting the need
% for systematic weight tuning.

% Bryson's Rule Initialization
[Q_Bry, R_Bry] = S_Bryson();     % Baseline LQG control weights

% Nominal Simulation with Bryson's Rule
[X_GT, X_est, u_out_k] = run_LQG_simulation(Q_Bry, R_Bry);

% Visualization
visualize_x;      % State trajectories
visualize_err;    % Error states and control inputs