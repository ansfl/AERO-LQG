%% Error analysis vs. time

l_w = 2.8;
O_3 = [0,0,0]; I_1 = [1,1,1];                % Utils
X_ref = [O_3, O_3, O_3, O_3]';               % Desired setpoint

err_est = [vecnorm(X_GT(1:3,:) - X_est(1:3,:)); vecnorm(X_GT(4:6,:) - X_est(4:6,:)); vecnorm(X_GT(7:9,:) - X_est(7:9,:)); vecnorm(X_GT(10:12,:) - X_est(10:12,:))];
err_ctr = [vecnorm(X_GT(1:3,:) - X_ref(1:3,:)); vecnorm(X_GT(4:6,:) - X_ref(4:6,:)); vecnorm(X_GT(7:9,:) - X_ref(7:9,:)); vecnorm(X_GT(10:12,:) - X_ref(10:12,:))];

Fig(fig_loc);
fs = 13;

% -------------- Position -------------- %
subplot(4, 3, 1); hold on; 
plot(tt, err_est(1,:), LineWidth=l_w/2);
ylabel('$\| {e}_{\xi^I} (t) \|$ [m]', fontsize=fs);

subplot(4, 3, 2); hold on;
plot(tt, err_ctr(1,:), LineWidth=l_w/2);
ylabel('$\| {\varepsilon}_{\xi^I} (t) \|$ [m]', fontsize=fs);

subplot(4, 3, 3); hold on;
plot(tt, u_out_k(1,:), LineWidth=l_w/2); ylabel('$ u_z (t)$ [N]', fontsize=fs);

% -------------- Velocity -------------- %
subplot(4, 3, 4); hold on; 
plot(tt, err_est(2,:), LineWidth=l_w/2); ylabel('$\| {e}_{v^I} (t) \|$ [m/s]', fontsize=fs);

subplot(4, 3, 5); hold on;
plot(tt, err_ctr(2,:), LineWidth=l_w/2); ylabel('$\| {\varepsilon}_{v^B} (t) \|$ [m/s]', fontsize=fs);

subplot(4, 3, 6); hold on;
plot(tt, u_out_k(2,:), LineWidth=l_w/2); ylabel('$ u_\phi (t) $ [N m]', fontsize=fs);

% -------------- Orientation -------------- %
wrap_to_pi = @(angle) mod(angle + pi, 2*pi) - pi;

subplot(4, 3, 7); hold on; 
plot(tt, (err_est(3,:))*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {e}_{\eta^I} (t) \|$ [$^\circ$]', fontsize=fs);

subplot(4, 3, 8); hold on;
plot(tt, (err_ctr(3,:))*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {\varepsilon}_{\eta^I} (t) \|$ [$^\circ$]', fontsize=fs);

subplot(4, 3, 9); hold on;
plot(tt, u_out_k(3,:), LineWidth=l_w/2); ylabel('$ {u}_{\theta} (t) $ [N m]', fontsize=fs);

% -------------- Angular rates -------------- %
subplot(4, 3, 10); hold on; 
plot(tt, err_est(4,:)*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {e}_{\omega^B} (t) \|$ [$^\circ$/s]', fontsize=fs);
xlabel('Time [s]', fontsize=fs);

subplot(4, 3, 11); hold on;
plot(tt, err_ctr(4,:)*Rad_2_deg, LineWidth=l_w/2); ylabel('$\| {\varepsilon}_{\omega^B} (t) \|$ [$^\circ$/s]', fontsize=fs);
xlabel('Time [s]', fontsize=fs);

subplot(4, 3, 12); hold on;
plot(tt, u_out_k(4,:), LineWidth=l_w/2); ylabel('${u}_{\psi} (t) $ [N m]', fontsize=fs);
xlabel('Time [s]', fontsize=fs);

% Uncomment to add / save image
% % legend({'$\boldmath{x}_{GT}$', '$\hat{\boldmath{x}}$'}, 'Interpreter', 'latex', 'Orientation', 'horizontal', 'Location', 'northoutside', 'FontSize', 12);
% exportgraphics(gcf, 'Fig_Instability_1.png', 'Resolution', 300);