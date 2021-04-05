yalmip('clear')
clear all; clc;
%parameters
x0 = [0.1 0.1 0.1 0.0 0.0 0.0 0.0]';    % initial state
dt = 0.1;                               % sampling rate
N_extra_steps = 300;                    % number of steps each run
global N_horizon
global R
global Q

%tuning
N_horizon = 5;                          % <5 does not work; then x_5 becomes >1 over time
P_gain = 1;                          % weight of terminal cost
Q = diag([500, 500, 500, 1e-7, 1, 1, 1]);
R = diag([20, 20, 20, 1]);

%flags
rr_suboptimal_MPC = 2;       % [0/1/2] dont run / run MPC / run suboptimal MPC
rr_non_lin_model = 0;        % [0/1]   use linearization / use nonlinear model
rr_solve_DARE = 2;           % [0/1/2] use P=I / DARE -> P / LQR infinite horizon optimal feedback

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