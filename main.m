%{
!YALMIP and MOSEK are needed to successfully run the code!
Main Matlab script used for simulation of the micro satellite. Part of
practical asignment of the Model Predictive Course (SC42145) of the Delft
University of Technology.

Written by:
- Menno Looman
- Danny Looman

With the flags defined in the first block of the main file the execution of
of the script can be set to: simulate no MPC / normal MPC / suboptimal MPC.
Also the model can be switched from linear model to nonlinear model. At
last the P that is used can be chosen.
%}
yalmip('clear'); clear all; clc;

% Flags
flag_MPC_type = 1;          % [0/1/2] dont run / run MPC / run suboptimal MPC
flag_model_type = 1;        % [0/1]   use linearization / use nonlinear model for simulation
flag_P_type = 2;            % [0/1/2] P=I (not recommended) / DARE -> P / LQR infinite horizon optimal feedback

% Tuning parameters
global N_horizon R Q
N_horizon = 5;                                  % Prediction horizon
beta = 10;                                      % Weight of terminal cost
Q = diag([500, 500, 500, 1e-7, 20, 20, 20]);    % State weight
R = diag([5, 5, 5, 1]);                         % Input weight

% Simulation parameters 
x0_quat = eul2quat([deg2rad(1.0), deg2rad(1.0), deg2rad(5.0)], 'XYZ'); 
x0 = [0.0 0.0 0.0 0.0 x0_quat(2:4)]';           % initial state
N_extra_steps = 200;                            % number of steps take during simulation

%% define system
def_sys;
if flag_P_type 
    solve_DARE; 
end

%% Run model with controller
if flag_model_type
    Run_non_lin; 
else  
    Run_lin; 
end

%% plot results
Plot_results;
