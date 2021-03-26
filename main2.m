%close all; 
clear all; clc;
%parameters
x0 = [0.1 0.0 0.0 0.0 0.0 0.0 0.0]';    % initial state
dt = 0.1;                               % sampling rate
N_horizon = 6;                          % <5 does not work; then x_5 becomes >1 over time
N_extra_steps = 500;                    % number of steps each run
P_gain = 1000;                          % weight of terminal cost
rr_suboptimal_MPC = 0;                  % run sys using suboptimal MPC or finite horizon MPC
rr_non_lin_model = 1;                   % run sys using the linearised or original system
rr_save_figures = 0;                    % flag if figure needs to be saved

% define system
def_sys
%--------------------------------------------------------------------------
%% build controller
if ~rr_suboptimal_MPC 
    % Finite horizon MPC
    Controller_MPC
else
    % Suboptimal MPC approach
    Controller_suboptimal_MPC %still doesnt work
end
%--------------------------------------------------------------------------
%% Run model with controller
if ~rr_non_lin_model
    %linear model
    Run_lin %not compatable, needs fix
else
    %nonlinear model
    Run_non_lin
end
%--------------------------------------------------------------------------
%% plot results
Plot_results
if rr_save_figures
    Save_figure
end