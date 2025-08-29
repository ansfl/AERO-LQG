%% Flight Analytics — Horizontal Dot Charts (one row per metric)
% Purpose
%   Visualize per-metric performance of multiple optimizers as compact,
%   horizontal dot charts (8 metrics × 1 row each).
%
% Inputs (example data below)
%   - model_names : cellstr of optimizer labels (same order across metrics)
%   - metrics     : cellstr of metric labels (LaTeX-ready)
%   - data        : 1×nMetrics cell; each cell is a 1×nModels numeric row
%
% Output
%   - Figure saved as 'Fig_Analytics_1.png' (300 DPI)

clc; close all;

%% Labels and data (example)
model_names = {'PS','BR','GA','BY','CMA'};   % 6 optimizers

metrics = { ...
    'Position RMSE [m]', ...
    'Orientation RMSE [$^\circ$]', ...
    'Recovery for a standardized gust [s]', ...
    'Average overshoot [$\%$]', ...
    'Actuator saturation fraction [$\%$]', ...
    'Power [W]', ...
    'Efficiency [-]', ...
    'Max Flight Time [min]'};

% Aggregated outcomes from extended optimization runs
data = { ...
    [0.11   0.53   0.22   0.07   0.01],  ... % Position RMSE
    [0.09   0.32   0.17   0.04   0.02],  ... % Orientation RMSE
    [1.62   2.32   1.21   1.92   2.21],  ... % Recovery
    [0.74   1.32   0.78   1.19   1.03],  ... % Overshoot
    [0.13   7.82   3.29   5.20   0.61],  ... % Saturation
    [77.8   81.2   75.1   74.3   71.4],  ... % Power
    [0.462  0.447  0.493  0.489  0.512], ... % Efficiency
    [25.9   23.7   26.3   25.7   28.2]};     % Max flight time


%% Basic checks
nMetrics = numel(metrics);
nModels  = numel(model_names);
assert(numel(data) == nMetrics, 'data must have one row per metric.');
assert(all(cellfun(@numel,data) == nModels), 'Each data row must match nModels.');

%% Figure styling

Fig([3000 700 550 450]);
l_w = 2.5; f_col = [0.8 0.4 0.1]; fs=10;

colors = lines(nModels);       % define colors for models


for i = 1:nMetrics
    subplot(nMetrics,1,i); hold on; grid on;
    
    % plot dots for this metric
    xvals = data{i};
    for j = 1:numel(xvals)
        scatter(xvals(j), 1, 70, 'filled', 'MarkerFaceColor', colors(j,:), 'MarkerEdgeColor', 'k');
    end
    
    yticks([]);                                 % hide y-axis (since only one row)
    xlabel(metrics{i},'FontWeight','bold', FontSize=fs); 
    set(gca,'YColor','none');                   % remove y-axis line
    xlim padded;
end

legend(model_names, 'FontSize', fs, 'Orientation','horizontal', 'NumColumns', ...
    min(nModels,5), 'Location','southoutside', 'Box','off');
% exportgraphics(gcf, 'Fig_Benchmark.png', 'Resolution', 300);