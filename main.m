close all; clear all; clear variables; clear variables; clc;
N_inputs = 4;

%% Initial state, input
x0 = [1 1 1 1 1 1 1]';%initial state
dt = 0.1; %sampling rate
N = 100; %number of time steps
t = 0:dt:dt*(N-1);
u = ones(N_inputs,N); %input at every time step

%% Continuous state space model
[A_con,B_con] = con_System();
D_con = zeros(7,N_inputs);
C_con = eye(7);
sys_con = ss(A_con,B_con,C_con,D_con);

%% Discretise system (less accurate then lsim)
sys_dis = c2d(sys_con,dt);
A_dis = sys_dis.A;
B_dis = sys_dis.B;

%% State constraints in the form Fx <= state_bounds
F = [eye(7); -eye(7)];
max_state_bounds = [1; 1; 1; 800; 1; 1; 1];
state_bounds = [max_state_bounds; -max_state_bounds];

%% Input constraints in the form Gu <= input_bounds
G = [eye(N_inputs); -eye(N_inputs)];
input_bounds = [0.0484; 0.0484; 0.0398; 0.0020; -0.0484; -0.0484; -0.0398; -0.0020];

%% Simulate system
[~,~,x_dis] = lsim(sys_con,u,t,x0);
%notice that x_dis is transposed

%% Objective Function LQR
Q = diag([500, 500, 500, 1e-7, 1, 1, 1]);
R = diag([200, 200, 200, 1]);
