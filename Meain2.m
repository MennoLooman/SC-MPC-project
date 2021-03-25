close all; clear all; clc;
%parameters
x0 = [0.1 0.0 0.0 0.0 0.0 0.0 0.0]';    % initial state
dt = 0.1;                               % sampling rate
N_horizon = 6;                          % <5 does not work; then x_5 becomes >1 over time
P_gain = 1000;                          % weight of terminal cost

% define system
def_sys
%--------------------------------------------------------------------------
%% MPC
Controller_MPC %not compatable, needs fix

%% Suboptimal MPC
Controller_suboptimal_MPC
%--------------------------------------------------------------------------
%% nonlinear model
%run controller using linear or nonlinear sys
N_extra_steps = 2500;
Run_non_lin

%% linear model
%run controller using linear or nonlinear sys
N_extra_steps = 2500;
x_save = x0;
Run_lin %not compatable, needs fix
%--------------------------------------------------------------------------
%% plot results
Plot_results

%% Save figure
Save_figure
