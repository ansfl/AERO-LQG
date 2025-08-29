%% CMA-ES (minimal) for LQG diagonal Q,R — main script
% Purpose
%   Tune diagonal Q and R for an LQG controller using a lightweight CMA-ES.
%   Decision vector: z = [log10 diag(Q); log10 diag(R)] to ensure positivity.
%
% Flow
%   1) Bryson's rule → seed scales (Q_Bry, R_Bry)
%   2) Define bounds in log10-space around the seed
%   3) Optimize an unconstrained vector y, map to box z via logistic
%   4) Rebuild Q,R from z, evaluate fitnessFcn(Q,R)
%   5) Simulate and visualize
%
% Notes
%   - Bounds here allow ±delta decades; widen/narrow as needed.
%   - This file is a script with local functions at the end (MATLAB ≥ R2016b).

%% -------- Seed and problem sizing --------
[Q_Bry, R_Bry] = S_Bryson();                 % Baseline LQG control weights

nx    = size(Q_Bry,1);                        % state dimension
nu    = size(R_Bry,1);                        % input dimension
nvars = nx + nu;                              % total decision variables

% Guard zeros for log10 and pack initial point (z0 in log10-space)
q0 = max(diag(Q_Bry), 1e-12);
r0 = max(diag(R_Bry), 1e-12);
z0 = [log10(q0); log10(r0)];

% -------- Bounds in log10-space (box) --------
% WARNING: deltaQ=10 spans 10 decades (very wide). Keep if intentional.
deltaQ = 10; deltaR = 10;       % typical: 2 → ±2 decades
lb = [z0(1:nx)          - deltaQ; z0(nx+1:end)          - deltaR];
ub = [z0(1:nx)          + deltaQ; z0(nx+1:end)          + deltaR];

% Unconstrained → box mapping via logistic
toBox = @(y) lb + (ub - lb) .* (1 ./ (1 + exp(-y)));

% Wrapped objective over y (maps y→z→Q,R)
obj_y = @(y) eval_cost(toBox(y), nx, nu, @fitnessFcn);

% Start y at the inverse-logistic of the mid-box (near Bryson)
mid = 0.5*(lb + ub);
y0  = log(mid - lb) - log(ub - mid);           % inverse of logistic on [lb,ub]

% -------- Run minimal CMA-ES --------
opts.popsize = 8 + floor(3*log(nvars));        % default pop-size rule
opts.maxiter = 100;                            % budget knob
opts.verbose = 1;                              % 1=print progress
opts.sigma0  = 0.5;                            % initial global step-size
[y_best, ~, history] = cmaes_basic(obj_y, y0, opts); %#ok<ASGLU>

% -------- Unpack best, simulate, visualize --------
z_star = toBox(y_best);
Q_opt  = diag(10.^z_star(1:nx));
R_opt  = diag(10.^z_star(nx+1:end));

[X_GT, X_est, u_out_k] = run_LQG_simulation(Q_opt, R_opt); %#ok<NASGU>
visualize_x;        % State trajectories
visualize_err;      % Error states and control inputs


%% ===================== Local functions =====================

function J = eval_cost(z, nx, nu, fitnessFcnHandle)
% EVAL_COST  Rebuild Q,R from log10-diagonal vector z and evaluate cost.
% Inputs
%   z                 [nx+nu,1]  (log10 diag(Q); log10 diag(R))
%   nx, nu            sizes of Q and R
%   fitnessFcnHandle  function handle: J = fitnessFcnHandle(Q, R)
% Output
%   J                 scalar objective value
    Q = diag(10.^z(1:nx));
    R = diag(10.^z(nx+1:nx+nu));
    J = fitnessFcnHandle(Q, R);
end


function [m, fbest, hist] = cmaes_basic(f, m0, opts)
% CMAES_BASIC  Minimal CMA-ES (rank-μ update; no restarts).
% Inputs
%   f     : @(x) -> scalar   objective (x is n×1)
%   m0    : n×1               initial mean in parameter space
%   opts  : struct            fields: popsize, maxiter, sigma0, verbose
% Outputs
%   m     : n×1               final mean (last iterate)
%   fbest : double            best objective value seen
%   hist  : struct            traces: .best (global best), .sigma, .f (best per iter)

    n = numel(m0);
    lambda  = max(4, opts.popsize);
    mu      = floor(lambda/2);
    w       = log(mu + 0.5) - log(1:mu)'; 
    w       = w / sum(w);
    mu_eff  = 1 / sum(w.^2);

    sigma = opts.sigma0;
    cs    = (mu_eff + 2)/(n + mu_eff + 5);
    ds    = 1 + cs + 2*max(0, sqrt((mu_eff-1)/(n+1)) - 1) + 0.3;
    cc    = (4 + mu_eff/n) / (n + 4 + 2*mu_eff/n);
    c1    = 2 / ((n + 1.3)^2 + mu_eff);
    cmu   = min(1 - c1, 2*(mu_eff - 2 + 1/mu_eff)/((n + 2)^2 + mu_eff));
    chi_n = sqrt(n)*(1 - 1/(4*n) + 1/(21*n^2));

    m   = m0(:);
    ps  = zeros(n,1);
    pc  = zeros(n,1);
    C   = eye(n);
    fbest = inf;
    hist  = struct('best',[],'sigma',[],'f',[]);

    for iter = 1:opts.maxiter
        % Decompose covariance
        [B,D] = eig((C + C')/2);
        D     = max(real(diag(D)), 1e-14);
        B     = real(B);
        A     = B * diag(sqrt(D));         % C^(1/2)

        % Sample and evaluate
        Z = randn(n, lambda);
        Y = A * Z;                         % correlated steps
        X = m + sigma * Y;                 % candidates

        fvals = zeros(lambda,1);
        for k = 1:lambda
            fvals(k) = f(X(:,k));
        end

        % Select μ best
        [f_sorted, idx] = sort(fvals);
        y_sel = Y(:, idx(1:mu));
        z_sel = Z(:, idx(1:mu));           % for ps update
        fbest_iter = f_sorted(1);
        if fbest_iter < fbest
            fbest = fbest_iter;
        end

        % Mean update
        m = m + sigma * (y_sel * w);

        % Evolution paths
        ps = (1 - cs) * ps + sqrt(cs*(2 - cs)*mu_eff) * (B * (z_sel * w));
        hsig = norm(ps) / sqrt(1 - (1 - cs)^(2*iter)) / chi_n < (1.4 + 2/(n+1));
        pc = (1 - cc) * pc + hsig * sqrt(cc*(2 - cc)*mu_eff) * (y_sel * w);

        % Covariance & step-size updates
        C     = (1 - c1 - cmu) * C + c1 * (pc*pc.') + cmu * (y_sel * diag(w) * y_sel.');
        sigma = sigma * exp((cs/ds) * (norm(ps)/chi_n - 1));

        % Trace
        if nargout > 2
            hist.best(end+1,1)  = fbest; %#ok<AGROW>
            hist.sigma(end+1,1) = sigma; %#ok<AGROW>
            hist.f(end+1,1)     = fbest_iter; %#ok<AGROW>
        end

        if isfield(opts,'verbose') && opts.verbose
            if mod(iter,10)==1 || iter==opts.maxiter
                fprintf('iter %4d  fbest %.4g  sigma %.3g\n', iter, fbest, sigma);
            end
        end
    end
end