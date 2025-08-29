%% Genetic Algorithm Optimization for LQG Weights
% This routine demonstrates how GA can be applied to tune diagonal Q and R
% weight matrices for an LQG controller. Bryson's rule provides the initial
% scaling, and optimization is performed in log10-space to preserve
% positivity and improve numerical conditioning.
%
% Workflow:
%   1) Initialize Q,R using Bryson's rule
%   2) Encode decision vector z = [log10 diag(Q); log10 diag(R)]
%   3) Define fitness function wrapper around run_LQG_simulation
%   4) Run MATLAB's GA optimizer with bounded search space
%   5) Reconstruct Q_opt, R_opt and simulate closed-loop behavior
%   6) Visualize resulting trajectories and errors

%% Baseline initialization (Bryson's Rule)
[Q_Bry, R_Bry] = S_Bryson();        % Baseline LQG control weights

%% Problem dimensions
nx = size(Q_Bry,1);                 % state dimension
nu = size(R_Bry,1);                 % input dimension
nvars = nx + nu;                    % total dimension of search vector

%% Fitness wrapper (decision variables → cost)
% Decision vector structure:
%   z = [log10 diag(Q); log10 diag(R)]
fitness = @(z) fitnessFcn( ...
    diag(10.^z(1:nx)), ...          % Q (diagonal, PSD)
    diag(10.^z(nx+1:end)));         % R (diagonal, PSD)

%% Decision vector initialization from Bryson's rule
q0 = max(diag(Q_Bry), 1e-12);       % guard against log(0)
r0 = max(diag(R_Bry), 1e-12);
z0 = [log10(q0); log10(r0)];

%% Bound constraints (search space)
% Allow ±2 decades around Bryson's rule scaling
%   → 10^(±2) ≈ [0.01, 100]
deltaQ = 2; deltaR = 2;
lb = [z0(1:nx) - deltaQ; z0(nx+1:end) - deltaR];
ub = [z0(1:nx) + deltaQ; z0(nx+1:end) + deltaR];

%% Run GA optimizer
opts = optimoptions('ga', ...
    'Display','iter', ...
    'PopulationSize', 50, ...
    'MaxGenerations', 5);

[z_opt, J_opt] = ga(fitness, nvars, [], [], [], [], lb, ub, [], opts);

%% Reconstruct optimized weights and simulate
Q_opt = diag(10.^z_opt(1:nx));
R_opt = diag(10.^z_opt(nx+1:end));

[X_GT, X_est, u_out_k] = run_LQG_simulation(Q_opt, R_opt);

%% Visualization
visualize_x;        % State trajectories
visualize_err;      % Error states and control inputs