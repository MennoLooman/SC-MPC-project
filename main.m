close all; clear all; clear variables; clear variables; clc;

%% Initial state, input
x0 = [1 1 1 1 1 1 1]';%initial state
dt = 0.1; %sampling rate
N = 100; %number of time steps
t = 0:dt:dt*(N-1);
u = ones(4,N); %input at every time step

%% Continuous state space model
[A_con,B_con] = con_System();
D_con = zeros(7,4);
C_con = eye(7);
sys_con = ss(A_con,B_con,C_con,D_con);

%% Discretise system (less accurate then lsim)
sys_dis = c2d(sys_con,dt);
A_dis = sys_dis.A;
B_dis = sys_dis.B;

%% Simulate system
[~,~,x_dis] = lsim(sys_con,u,t,x0);
%notice that x_dis is transposed