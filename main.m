%close all; 
yalmip('clear')
clear all; clc;
global N_horizon
%parameters
x0 = [0.1 0.1 0.1 0.0 0.0 0.0 0.0]';    % initial state
dt = 0.1;                               % sampling rate
N_horizon = 10;                          % <5 does not work; then x_5 becomes >1 over time
N_extra_steps = 200;                    % number of steps each run
P_gain = 1;                          % weight of terminal cost
%flags
rr_suboptimal_MPC = 0;                  % run sys using suboptimal MPC or finite horizon MPC
rr_non_lin_model = 0;                   % run sys using the linearised or original system
rr_plot_non_lin = 0;                    % run input generated using linearization on nonlinear model
rr_solve_DARE = 1;                      % LQR infinite horizon optimal feedback and cost
rr_save_figures = 0;                    % flag if figure needs to be saved

% define system
def_sys;
if rr_solve_DARE, solve_DARE; end
%% debug
Run_MPC_debug;
%% plot debug
Plot_results;
%--------------------------------------------------------------------------
%% build controller
if ~rr_suboptimal_MPC 
    % Finite horizon MPC
    Controller_MPC;
else
    % Suboptimal MPC approach
    Controller_suboptimal_MPC; %still doesnt work
end
[solutions,diagnostics] = Controller({x0,1});
value(u_tot)
%--------------------------------------------------------------------------
%% Run model with controller
if ~rr_non_lin_model
    %linear model
    Run_lin;
    if rr_plot_non_lin, plot_non_lin; end 
else
    %nonlinear model
    Run_non_lin;
end
%--------------------------------------------------------------------------
%% plot results
Plot_results;
if rr_save_figures, Save_figure; end %outdated