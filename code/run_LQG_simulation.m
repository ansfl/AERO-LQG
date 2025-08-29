function [x_k, x_e, u_out_k] = run_LQG_simulation( Q_lqr, R_lqr )
%RUN_LQG_SIMULATION  Simulate closed-loop quadrotor with LQG control.
%
%   [x_k, x_e, u_out_k] = run_LQG_simulation(Q_lqr, R_lqr)
%
%   Purpose:
%       Executes a nonlinear quadrotor simulation under LQG control,
%       combining LQR feedback (based on Q, R weights) with a Kalman
%       filter for state estimation. Returns both the true trajectory
%       and the estimated states.
%
%   Inputs:
%       Q_lqr    - State weighting matrix (for LQR design)
%       R_lqr    - Control weighting matrix (for LQR design)
%
%   Outputs:
%       x_k      - Ground-truth system state trajectory
%       x_e      - Estimated system state trajectory (Kalman filter)
%       u_out_k  - Saturated actuator command history
%
%   Notes:
%       - Simulation uses nonlinear dynamics (fx_Quad) integrated via RK.
%       - Estimation loop runs with prediction and periodic correction.
%       - Actuator saturation is enforced through u_sat().
%
% --------------------------------------------------------------

%%
% ----------- Initial Condition ------------ % (Hovering setpoint)
O_3 = [0,0,0]; I_1 = [1,1,1];                % Utils
X_ref = [O_3, O_3, O_3, O_3]';               % Desired setpoint
x_0   = [I_1*0.1, O_3, O_3+0.1, O_3]';          % Initial point
rho = 1;                                     % Prediction-Update frequency / ratio

% System initialization
[A, B, dt, tt, t_f, t_tau, Rad_2_RPM, x_dim, u_dim, W, V] = sys_init();

% Riccati solution (based on Q and R input / optimization variables)
K = lqr(A, B, Q_lqr, R_lqr);

% System parameters
[m, L, g, J_xx, J_yy, J_zz, k_T, k_M, eta_eff, Mat_Mix, w_hover, T_max] = sys_params();

% Initial conditions
u_out_k = zeros(u_dim, length(tt)); 
u_cmd_k = zeros(u_dim, length(tt));
x_std = zeros(x_dim, length(tt)); y_k = x_std;
x_k = x_std; x_k(:,1) = x_0;
x_e = x_std; x_e(:,1) = x_0;

x_idx = (1:x_dim)';                             % Full obsverability
v_idx = [4,5,6]';
w_k = @() randn( size(A*x_k(:,1)) ).*sqrt(diag(W));
v_k = @() randn( size(x_idx) ).*sqrt(diag(V));

% Solution loop
A_k = expm(A*dt); B_k = B*dt;                   % Discrete-time solution
P_k = 0.1*eye(x_dim);                           % Initialize error covariance

for k=2:(length(tt))
    % -------------- Control module ------------- % (x_k == GT | x_e == x^est)
    u_cmd_k(:,k) = u_sat( -K*(x_e(:,k-1) - X_ref), L, J_zz, T_max );  % Control command
    u_out_k(:,k) = u_out_k(:,k-1) + t_tau * (u_cmd_k(:,k) - u_out_k(:,k-1) );
    
    % ----------------- Process ----------------- %
    x_k(:,k) = f_RK(@fx_Quad, x_k(:,k-1), u_out_k(:,k), dt) + w_k();% Nonlinear process (GT)
    
    % ------------ State estimation ------------- %
    % --------------- Prediction ---------------- %
    x_e(:,k) = A_k*x_e(:,k-1) + B_k*u_out_k(:,k); % Linear predictor (x'=[A-BK]x)
    P_k = A_k*P_k*A_k' + W;                       % Error state covariance (open-loop)

    % ------------- Position Update ------------- %
    if mod(k,rho) == 0
        C_k = h_x(x_idx, x_dim);                  % Projection: position, vel, gyros
        y_meas = C_k*x_k(:,k-1) + v_k();          % Outputs (Noisy measurements)
        y_pred = C_k*x_e(:,k-1);                  % Predicted measurements
        y_res  = y_meas - y_pred;                 % Measurements residual

        [x_e(:,k), P_k] = KF_update(x_e(:,k-1), P_k, y_res, C_k, V); % Kalman filter (loop-closure)
        y_k(x_idx,k) = y_meas;
    end
end