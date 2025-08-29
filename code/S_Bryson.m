function [Q_ii, R_jj] = S_Bryson( )
% ======================== Description ======================== %
%                                                               %
%   Purpose : Generate state (Q) and control (R) weighting      %
%             matrices based on Bryson's rule.                  %
%                                                               %
%   Input   : None (all parameters defined internally)          %
%   Output  : Q, R  - Weighting matrices for LQR/LQG design     %
%                                                               %
% ========================== Content ========================== %

I_3 = ones(1,3);

% Acceptable (max) state deviations
x_pos = 0.1;                         % [m]
x_vel = 0.2;                         % [m/s]
x_ori = 0.1;                         % [rad]
x_ang = 1.0;                         % [rad/s]
% ---------------------------------- %
Q_ii = diag([x_pos*I_3, x_vel*I_3, x_ori*I_3, x_ang*I_3])^(-2);

% Acceptable (max) control deviations
u_z = 19.0;                          % [N] T_max = mg/2 [N]
u_p = 0.3;                           % [N*m] 0.02
u_q = 0.3;                           % [N*m] 0.02
u_r = 0.1;                           % [N*m] 0.05
% ---------------------------------- %
R_jj = diag([u_z, u_p, u_q, u_r])^(-2);