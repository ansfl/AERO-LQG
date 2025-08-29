% ======================== Description ======================== %
%                                                               %
%   Purpose : Define initial conditions for system simulation,  %
%             including state, control, and covariance setup.   %
%                                                               %
%   Input   : N/A (all parameters are assumed predefined)       %
%   Output  : Initialized state, control, and covariance arrays %
%                                                               %
% ========================== Content ========================== %

% Initialization of control vectors
u_out_k = zeros(u_dim, length(tt)); 
u_cmd_k = zeros(u_dim, length(tt));

% Initialization of state vectors
x_std = zeros(x_dim, length(tt)); y_k = x_std;
x_k = x_std; x_k(:,1) = x_0;
x_e = x_std; x_e(:,1) = x_0;

x_idx = (1:x_dim)';                             % Full obsverability
v_idx = [4,5,6]';
w_k = @() randn( size(A*x_k(:,1)) ).*sqrt(diag(W));
v_k = @() randn( size(x_idx) ).*sqrt(diag(V));

% Solution loop
A_k = expm(A*dt); B_k = B*dt;                % Discrete-time solution
P_k = 0.1*eye(x_dim);                        % Initialize error covariance