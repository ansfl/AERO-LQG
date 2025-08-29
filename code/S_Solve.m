% ======================== Description ======================== %
%                                                               %
%   Purpose : Simulate closed-loop quadrotor dynamics with LQG  %
%             control and Kalman-based state estimation.        %
%                                                               %
%   Input   : N/A (all parameters and states are assumed given) %
%   Output  : State vector trajectory and associated covariance %
%                                                               %
% ========================== Content ========================== %

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

    % Save root diagonal (standard deviation)
    x_std(:,k) = sqrt(diag(P_k));
end