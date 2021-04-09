yalmip('clear'); clear all; clc;

% parameters
% initial quaternion based on euler angles in order: roll - pitch - yaw
x0_quat = eul2quat([deg2rad(1.0), deg2rad(1.0), deg2rad(5.0)], 'XYZ');
x0 = [0.0 0.0 0.0 0.0 x0_quat(2:4)]';   % initial state
dt = 0.1;                               % sampling rate
N_extra_steps = 200;                    % number of steps each run
global N_horizon
global R
global Q

%tuning
N_horizon = 5;                          % 
P_gain = 10;                          % weight of terminal cost
Q = diag([500, 500, 500, 1e-7, 20, 20, 20]);
R = diag([5, 5, 5, 1]);

%flags
rr_suboptimal_MPC = 2;       % [0/1/2] dont run / run MPC / run suboptimal MPC
rr_non_lin_model = 1;        % [0/1]   use linearization / use nonlinear model
rr_solve_DARE = 1;           % [0/1/2] use P=I / DARE -> P / LQR infinite horizon optimal feedback

%% define system
def_sys;
if rr_solve_DARE, solve_DARE; end
%--------------------------------------------------------------------------
%% Run model with controller
if ~rr_non_lin_model
    %linear model
    Run_lin;
else
    %nonlinear model
    Run_non_lin;
end
%--------------------------------------------------------------------------
%% plot results
Plot_results;