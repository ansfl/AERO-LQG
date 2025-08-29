%% Cost Function J_out — Surface & Contour Plot
%
% This script evaluates the scalar cost function J_out across a grid of
% scaled Bryson-style LQG weights (Q, R). For each grid point, the (1:3,1:3)
% block of Q and R is modified to ensure positive semi-definiteness, and
% the cost is computed via fitnessFcn().
%
% The result is visualized as a 3D surface overlaid with contour lines,
% where the Z-axis shows the cost function and the color encodes log10(J).
% This helps reveal cost trends and regions of convergence or divergence
% in the (Q, R) weight space.

%% Parameter grids
[Q_min, Q_step, Q_max] = deal(-10, 0.5, +10.0);
[R_min, R_step, R_max] = deal(-10, 0.5, +10.0);
[Q_n, R_n] = deal(Q_min:Q_step:Q_max, R_min:R_step:R_max);

Q_vals = Q_min:Q_step:Q_max;
R_vals = R_min:R_step:R_max;

% Nudge away from exact zeros to avoid singularities in downstream ops
nonzero_eps = 1e-3 * randn();
Q_vals = Q_vals + nonzero_eps;
R_vals = R_vals + nonzero_eps;

% Preallocate cost matrix
J = zeros(numel(Q_vals), numel(R_vals));

% Sweep over grid
for i = 1:numel(Q_vals)
    for j = 1:numel(R_vals)
        % LQG weights (Bryson-style baseline)
        [Q_Bry, R_Bry] = S_Bryson();    % user-defined function

        % Enforce PSD on the (1:3,1:3) blocks via L * L' construction
        QL = Q_Bry(1:3,1:3) * Q_vals(i);
        RL = R_Bry(1:3,1:3) * R_vals(j);

        Q_Bry(1:3,1:3) = QL * QL.';
        R_Bry(1:3,1:3) = RL * RL.';

        % Evaluate cost
        J_out = fitnessFcn(Q_Bry, R_Bry);   % user-defined function
        J(i, j) = J_out;
    end
    fprintf('(row %d/%d) last cost: %.6g\n', i, numel(Q_vals), J_out);
end


%% Sufrace plot visualization

fs = 18;
Fig(fig_loc);
[Q_grid, R_grid] = meshgrid(Q_vals, R_vals);

J_log = log10(J);                % transpose to match meshgrid orientation
hold on;
h_1 = surf(Q_grid, R_grid, J', J_log', 'EdgeAlpha',0.35,'FaceAlpha',0.8, 'FaceColor','interp');
colormap turbo; 
h_2 = contourf(Q_grid, R_grid, J_log, 50); %C,'EdgeAlpha',0.35,'FaceAlpha',0.8, 'FaceColor','interp');  % Z = J, color = log10(J)
hold off;

xlabel('$\| \mathbf{Q}_{\xi} \|_2$', fontsize=fs); ylabel('$\| \mathbf{R}_{\xi} \|_2$', fontsize=fs); zlabel('$\mathcal{J}_{out}$', fontsize=fs);
set(gca,'ZScale','log')              % keep geometric surface on log z-axis
view(45,30)

% exportgraphics(gcf, 'Fig_Cost_1.png', 'Resolution', 300);
