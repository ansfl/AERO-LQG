%% Particle Swarm Optimization for LQG Weights
% This routine applies PSO to tune diagonal Q and R for an LQG controller.
% Bryson's rule seeds the scaling, and optimization is performed in
% log10-space to ensure positivity and good conditioning.
%
% Workflow:
%   1) Initialize Q,R via Bryson's rule
%   2) Encode z = [log10 diag(Q); log10 diag(R)]
%   3) Define fitness wrapper around your simulation-based cost
%   4) Seed a swarm near Bryson and run PSO within bounds
%   5) Reconstruct Q_opt, R_opt and simulate
%   6) Visualize trajectories and errors

%% Baseline initialization (Bryson's Rule)
[Q_Bry, R_Bry] = S_Bryson();           % Baseline LQG control weights

% Problem dimensions
nx    = size(Q_Bry,1);                  % state dimension
nu    = size(R_Bry,1);                  % input dimension
nvars = nx + nu;                        % total decision variables

% Fitness wrapper (decision variables → cost)
% Decision vector: z = [log10 diag(Q); log10 diag(R)]
fitness = @(z) fitnessFcn( ...
    diag(10.^z(1:nx)), ...              % Q (diagonal, PSD)
    diag(10.^z(nx+1:end)));             % R (diagonal, PSD)

% Decision vector initialization from Bryson's rule
q0 = max(diag(Q_Bry), 1e-12);           % guard against log(0)
r0 = max(diag(R_Bry), 1e-12);
z0 = [log10(q0); log10(r0)];

% Bound constraints (search space)
% Allow ±2 decades around Bryson's scaling (10^(±2) ≈ [0.01, 100])
deltaQ = 2;  
deltaR = 2;
lb = [z0(1:nx) - deltaQ; z0(nx+1:end) - deltaR];
ub = [z0(1:nx) + deltaQ; z0(nx+1:end) + deltaR];

% Objective wrapper for particleswarm
obj = @(z) fitnessFcn( ...
    diag(10.^z(1:nx)), ...
    diag(10.^z(nx+1:end)));

% Swarm seeding near Bryson
SwarmSize = 40;                                              % budget knob
init = repmat(z0.', SwarmSize, 1) + 0.10*randn(SwarmSize, nvars); % small jitter
init = min(max(init, lb.'), ub.');                           % clip to bounds

% PSO options
opts = optimoptions('particleswarm', ...
    'SwarmSize',           SwarmSize, ...
    'MaxIterations',       10, ...       % increase if runtime allows
    'InitialSwarmMatrix',  init, ...
    'Display',             'iter', ...
    'UseParallel',         false);       % set true if fitnessFcn is thread-safe

% Run PSO
[z_star, fval, exitflag, out] = particleswarm(obj, nvars, lb, ub, opts); %#ok<ASGLU>

% Reconstruct optimized weights and simulate
Q_opt = diag(10.^z_star(1:nx));
R_opt = diag(10.^z_star(nx+1:end));

[X_GT, X_est, u_out_k] = run_LQG_simulation(Q_opt, R_opt);

% Visualization
visualize_x;        % State trajectories
visualize_err;      % Error states and control inputs