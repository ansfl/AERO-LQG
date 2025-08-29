%% MAIN.m — LQG Weight Optimization Launcher
% Purpose: Initialize system, set plotting defaults, and dispatch an optimizer.
% Requires: sys_init.m, sys_params.m, and one of Opt_{MT,BR,PSO,GA,BO,CMA_ES}.m on path.
% Output:   Optimizer scripts handle their own results/plots/workspace variables.

clc; clear; close all;

%% Graphics defaults (clean, LaTeX-ready figures)
set(0,'DefaultFigureRenderer','painters');
set(0,'defaultfigurecolor',[1 1 1]);
set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on');
set(groot,'DefaultLineLineWidth',2);
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultAxesTickLabelInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');

Fig = @(pos) figure('Renderer','painters','Position',pos);   % quick fig helper
fig_loc = [500 700 550 450];

%% System initialization
[A,B,dt,tt,t_f,t_tau,Rad_2_RPM,x_dim,u_dim,W,V] = sys_init(); %#ok<ASGLU>
Rad_2_deg = 360/(2*pi);        % radians → degrees
RPM_2_Rad = 2*pi/60;           % rpm → rad/s

%% System parameters
[m,L,g,J_xx,J_yy,J_zz,k_T,k_M,eta_eff,Mat_Mix,w_hover,T_max] = sys_params(); %#ok<ASGLU>

%% Optimizer Menu & Dispatcher (script-based)
% Presents a compact menu and runs the selected optimizer script.
% Expects one of: Opt_MT.m, Opt_BR.m, Opt_PSO.m, Opt_GA.m, Opt_BO.m, Opt_CMA_ES.m

fprintf([ ...
    '\nChoose Optimizer:\n' ...
    '  1) Manual tuning (MT)\n' ...
    '  2) Bryson''s Rule (BR)\n' ...
    '  3) Particle Swarm Optimization (PSO)\n' ...
    '  4) Genetic Algorithm (GA)\n' ...
    '  5) Bayesian Optimization (BY)\n' ...
    '  6) Covariance Matrix Adaptation (CMA-ES)\n']);

choice = input('Enter 1-6: ');
if ~isscalar(choice) || ~isfinite(choice) || choice~=fix(choice) || choice < 1 || choice > 6
    error('Choice must be an integer in 1..6.');
end

names = { 'Manual tuning (MT)', 'Bryson''s Rule (BR)', 'Particle Swarm Optimization (PSO)', ...
    'Genetic Algorithm (GA)', 'Bayesian Optimization (BO)', 'Covariance Matrix Adaptation (CMA-ES)'};

files = {'Opt_MT','Opt_BR','Opt_PSO','Opt_GA','Opt_BY','Opt_CMA_ES'};

[label, file] = deal(names{choice}, files{choice});

% Verify the script is reachable on the path
if ~(exist([file '.m'],'file') == 2)
    error('Selected script "%s.m" not found on the MATLAB path.', file);
end

%% Dispatch
banner(label);
t0 = tic;
try
    run(file);                      % executes e.g., Opt_PSO.m
    ok = true;  msg = '';
catch ME
    ok = false; msg = ME.message;   % or ME.getReport('extended','hyperlinks','off')
end
elapsed_s = toc(t0);

if ok
    fprintf('[OK]   %s completed in %.3f s\n\n', label, elapsed_s);
else
    warning('[FAIL] %s: %s', label, msg);
    fprintf('[FAIL] %s after %.3f s\n\n', label, elapsed_s);
end

% ---- Local helper ----
function banner(txt)
    pad = 6; rule = repmat('=', 1, max(48, numel(txt) + 2*pad));
    fprintf('\n%s\n>> %s <<\n%s\n', rule, txt, rule);
end

%% Cost Function J_out — Surface & Contour Plot
% visuazlize_cost                   % <-- Uncomment to execute