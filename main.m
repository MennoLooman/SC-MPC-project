close all; clear all; clc;
N_inputs = 4;
N_states = 7;

%% Initial state, input
% state x = [w_1 w_2 w_3 w_w e_1 e_2 e_3]';
% w_i : angular velocity of the body frame relative to the orbit frame
% w_w : angular velocity of the wheel relative to the body
% e_i : Euler parameters (toghether with n)
% input u = [t_1 t_2 t_3 t_w]';
% t_i : torque provided from thrusters
% t_w : internal axial torques applied by the platform to the wheel

x0 = [0.1 0.0 0.0 0.0 0.0 0.0 0.0]';%initial state
dt = 0.1; %sampling rate
N_steps = 1000; %number of time steps
t = 0:dt:dt*(N_steps-1);

%% Continuous state space model
[A_con,B_con] = con_System();
D_con = zeros(N_states,N_inputs);
C_con = eye(N_states);
sys_con = ss(A_con,B_con,C_con,D_con);

%% Discretise system (less accurate then lsim)
sys_dis = c2d(sys_con,dt);
A_dis = sys_dis.A;
B_dis = sys_dis.B;

%% Simulate system
u = ones(N_inputs,N_steps); %input at every time step
[~,~,x_dis] = lsim(sys_con,u,t,x0);
%notice that x_dis is transposed

%% State constraints in the form Fx <= state_bounds
F = [eye(N_states); -eye(N_states)];
max_state_bounds = [1; 1; 1; 800; 1; 1; 1];
state_bounds = [max_state_bounds; max_state_bounds];

%% Input constraints in the form Gu <= input_bounds
G = [eye(N_inputs); -eye(N_inputs)];
input_bounds = [0.0484; 0.0484; 0.0398; 0.0020; 0.0484; 0.0484; 0.0398; 0.0020];

%% Objective Function LQR
Q = diag([500, 500, 500, 1e-7, 1, 1, 1]);
R = diag([200, 200, 200, 1]);

N_horizon = 6; % <5 does not work; then x_5 becomes >1 over time
P = 1*eye(N_states);
x0_var = sdpvar(7,1);
x = sdpvar(repmat(N_states,1,N_horizon+1),repmat(1,1,N_horizon+1));
u = sdpvar(repmat(N_inputs,1,N_horizon), repmat(1,1,N_horizon));

Constraints = [x{1}==x0_var]; 
Objective = x{end}'*P*x{end};

for k = 1:N_horizon
   Objective = Objective + 0.5*( x{k}'*Q*x{k} + u{k}'*R*u{k} );
   Constraints = [Constraints, G*u{k} <= input_bounds, F*x{k} <= state_bounds];
   Constraints = [Constraints, x{k+1} == A_dis*x{k} + B_dis*u{k}];
end

Controller = optimizer(Constraints,Objective,[],x0_var,u{1});

%% Run Controller
x_save = zeros(N_states, N_steps);
u_save = zeros(N_inputs, N_steps);
x = x0;
for i = 1:N_steps
    u = Controller{x};
    u_save(:,i) = u;
    x = A_dis * x + B_dis * u;
    x_save(:,i) = x;
end

%% plot results
fig1 = figure();
tiledlayout(2,1)

% plot inputs
nexttile
hold on
for i = 1:N_inputs
    plot(t,u_save(i,:),'DisplayName',"u_"+num2str(i))
end
hold off
legend
title('Inputs over time')

% plot states
nexttile
hold on
for i = 1:N_states
    plot(t,x_save(i,:),'DisplayName',"x_"+num2str(i))
end
hold off
legend
title('States over time')

%save figure
figfile = fullfile("figures", "N_step="+num2str(N_steps)+"_N_hor="+num2str(N_horizon));
saveas(fig1,figfile);