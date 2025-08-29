%% -------------------------------------------- %
% --------- Analysis & Visualization ---------- %
% --------------------------------------------- %

l_w = 2.5; f_col = [0.8 0.4 0.1];
Fig(fig_loc);

% -------------- Position -------------- %
subplot(4, 3, 1); hold on; 
plot(tt, X_GT(1,:));
ylabel('$\xi_x^I(t)$ [m]', fontsize=14);

subplot(4, 3, 2); hold on;
plot(tt, X_GT(2,:));
ylabel('$\xi_y^I(t)$ [m]', fontsize=14);

subplot(4, 3, 3); hold on;
plot(tt, X_GT(3,:));
ylabel('$\xi_z^I(t)$ [m]', fontsize=14);

y_all = X_GT(1:3, :); ylim_shared = [min(y_all(:)), max(y_all(:))];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 1:3);

% -------------- Velocity -------------- %
subplot(4, 3, 4); hold on; 
plot(tt, X_GT(4,:));
ylabel('$v_x^B(t)$ [m/s]', fontsize=14);

subplot(4, 3, 5); hold on;
plot(tt, X_GT(5,:));
ylabel('$v_y^B(t)$ [m/s]', fontsize=14);

subplot(4, 3, 6); hold on;
plot(tt, X_GT(6,:));
ylabel('$v_z^B(t)$ [m/s]', fontsize=14);

y_all = X_GT(4:6, :); ylim_shared = [min(y_all(:)), max(y_all(:))];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 4:6);

% -------------- Orientation -------------- %
wrap_to_pi = @(angle) mod(angle + pi, 2*pi) - pi;

subplot(4, 3, 7); hold on; 
plot(tt, wrap_to_pi(X_GT(7,:))*Rad_2_deg); % plot(tt, X_est(7,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='-.');
% plot(tt, wrap_to_pi(X_est(7,:))*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\phi(t)$ [$\circ$]', fontsize=14);

subplot(4, 3, 8); hold on;
plot(tt, wrap_to_pi(X_GT(8,:))*Rad_2_deg); % plot(tt, X_est(8,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='-.');
% plot(tt, wrap_to_pi(X_est(8,:))*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\theta(t)$ [$\circ$]', fontsize=14);

subplot(4, 3, 9); hold on;
plot(tt, wrap_to_pi(X_GT(9,:))*Rad_2_deg); % plot(tt, X_est(9,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='-.');
% plot(tt, wrap_to_pi(X_est(9,:))*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\psi(t)$ [$\circ$]', fontsize=14);

y_all = X_GT(7:9, :); ylim_shared = [min(y_all(:))*Rad_2_deg, max(y_all(:))*Rad_2_deg];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 7:9);

% -------------- Angular rates -------------- %
subplot(4, 3, 10); hold on; 
plot(tt, X_GT(10,:)*Rad_2_deg); % plot(tt, X_est(10,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\omega_x^B(t)$ [$\circ$/s]', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 3, 11); hold on;
plot(tt, X_GT(11,:)*Rad_2_deg); % plot(tt, X_est(11,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\omega_y^B(t)$ [$\circ$/s]', fontsize=14);
xlabel('Time [s]', fontsize=14);

subplot(4, 3, 12); hold on;
plot(tt, X_GT(12,:)*Rad_2_deg); % plot(tt, X_est(12,:)*Rad_2_deg, color=f_col, LineWidth=l_w, LineStyle='--');
ylabel('$\omega_z^B(t)$ [$\circ$/s]', fontsize=14);
xlabel('Time [s]', fontsize=14);

y_all = X_GT(10:12, :); ylim_shared = [min(y_all(:))*Rad_2_deg, max(y_all(:))*Rad_2_deg];
arrayfun(@(i) set(subplot(4,3,i), 'YLim', ylim_shared), 10:12);
arrayfun(@(i) yline(subplot(4,3,i), 0, color='k', LineWidth=l_w/2, LineStyle='-.', alpha=0.8), 1:12);
arrayfun(@(i) grid(subplot(4, 3, i), 'on'), 1:12);

% Uncomment to add / save image
% legend({'$\boldmath{x}_{GT}$', '$\hat{\boldmath{x}}$'}, 'Interpreter', 'latex', 'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', 12);
% exportgraphics(gcf, 'Fig_States_1.png', 'Resolution', 300); % 300 DPI for high resolution
