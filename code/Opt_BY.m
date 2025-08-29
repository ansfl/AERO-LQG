%% Bayesian Optimization for LQG Weights (diagonal Q,R, log10-space)
% Purpose
%   Tune diagonal Q and R for an LQG controller via Bayesian Optimization.
%   Decision vector: z = [log10 diag(Q); log10 diag(R)] to enforce positivity.
%
% Workflow
%   1) Seed with Bryson's rule (Q_Bry, R_Bry)
%   2) Define bounded variables in log10-space (±2 decades)
%   3) BO objective maps table row → z → Q,R → fitness
%   4) Extract best point, rebuild Q_opt, R_opt
%   5) Simulate and visualize

%% Baseline initialization (Bryson's Rule)
[Q_Bry, R_Bry] = S_Bryson();                 % Baseline LQG control weights

% Problem dimensions
nx    = size(Q_Bry,1);                       % state dimension
nu    = size(R_Bry,1);                       % input dimension
nvars = nx + nu;                              %#ok<NASGU> % total decision variables

% Initial point in log10-space (guard against log(0))
q0 = max(diag(Q_Bry), 1e-12);
r0 = max(diag(R_Bry), 1e-12);
z0 = [log10(q0); log10(r0)];

% Bounds: ±2 decades around Bryson (tune as needed)
deltaQ = 2; 
deltaR = 2;
lb = [z0(1:nx)          - deltaQ; z0(nx+1:end)          - deltaR];
ub = [z0(1:nx)          + deltaQ; z0(nx+1:end)          + deltaR];

% Optimizable variables (q1..q_nx, r1..r_nu)
qVars = arrayfun(@(i) optimizableVariable(sprintf('q%d',i), [lb(i) ub(i)]), ...
                 1:nx, 'UniformOutput', false);
rVars = arrayfun(@(j) optimizableVariable(sprintf('r%d',j), [lb(nx+j) ub(nx+j)]), ...
                 1:nu, 'UniformOutput', false);
vars  = [qVars{:} rVars{:}];

% Start BO at Bryson's rule
InitialX = array2table(z0.', 'VariableNames', {vars.Name});

% Objective: table row → z → Q,R → fitness
objFcn = @(T) obj_from_table(T, nx, nu, @fitnessFcn);

% Run Bayesian Optimization
results = bayesopt(objFcn, vars, ...
    'InitialX',                 InitialX, ...
    'AcquisitionFunctionName',  'expected-improvement-plus', ...
    'IsObjectiveDeterministic', true, ...   % set false if simulation is noisy
    'MaxObjectiveEvaluations',  60, ...
    'UseParallel',              false, ...
    'Verbose',                  1);

% Best point → Q_opt, R_opt
bestTbl = bestPoint(results);
z_best  = bestTbl{:,:}.';                    % numeric row → column vector
Q_opt   = diag(10.^z_best(1:nx));
R_opt   = diag(10.^z_best(nx+1:end));

% Simulate and visualize
[X_GT, X_est, u_out_k] = run_LQG_simulation(Q_opt, R_opt); %#ok<NASGU>
visualize_x;        % State trajectories
visualize_err;      % Error states and control inputs


%% ----------------- Local helper (keeps objFcn tidy) -----------------
function J = obj_from_table(T, nx, nu, fitnessFcnHandle)
% OBJ_FROM_TABLE  Convert BO table row to Q,R and evaluate cost.
% Inputs
%   T                 1×(nx+nu) table of log10 variables
%   nx, nu            sizes of Q and R
%   fitnessFcnHandle  function handle: J = fitnessFcnHandle(Q, R)
% Output
%   J                 scalar objective value
    zrow   = table2array(T(1,:));      % 1×(nx+nu)
    q_logs = zrow(1:nx);
    r_logs = zrow(nx+1:nx+nu);

    Q = diag(10.^q_logs);
    R = diag(10.^r_logs);

    J = fitnessFcnHandle(Q, R);
end