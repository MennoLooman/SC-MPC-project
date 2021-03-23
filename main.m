close all; clear all; clc;
N_inputs = 4;
N_states = 7;

%% Initial state, input
% state x = [w_1 w_2 w_3 w_w e_1 e_2 e_3]';
% - w_i : angular velocity of the body frame relative to the orbit frame
% - w_w : angular velocity of the wheel relative to the body
% - e_i : Euler parameters (toghether with n)
% - input u = [t_1 t_2 t_3 t_w]';
% - t_i : torque provided from thrusters
% - t_w : internal axial torques applied by the platform to the wheel

x0 = [0.1 0.0 0.0 0.0 0.0 0.0 0.0]';    % initial state
dt = 0.1;                               % sampling rate
N_horizon = 6;                          % <5 does not work; then x_5 becomes >1 over time
N_steps = 500;                          % number of time steps
t = 0:dt:dt*(N_steps-1);

%% Continuous state space model
[A_con,B_con] = con_System();
D_con = zeros(N_states,N_inputs);
C_con = eye(N_states); % idea: change C to only measure first 4 physicall states (sys still observable)
sys_con = ss(A_con,B_con,C_con,D_con);

clear D_con C_con

%% Discretise system (less accurate then lsim)
sys_dis = c2d(sys_con, dt, 'zoh');
A_dis = sys_dis.A;
B_dis = sys_dis.B;

% Simulate discrete system
u = ones(N_inputs,N_steps); %input at every time step
[~,~,x_dis] = lsim(sys_con,u,t,x0);
x_dis = x_dis';

%% Constraints definition
% State constraints in the form Fx <= state_bounds
F = [eye(N_states); -eye(N_states)];
max_state_bounds = [1; 1; 1; 800; 1; 1; 1];
state_bounds = [max_state_bounds; max_state_bounds];

% Input constraints
int_in_bounds = diag([0.0484; 0.0484; 0.0398]); %0.0020

%% Objective Function LQR
Q = diag([500, 500, 500, 1e-7, 1, 1, 1]);
R = diag([200, 200, 200, 1]);

P_gain = 1000; % weight of terminal cost
P = P_gain*eye(N_states);
x0_var = sdpvar(7,1);
x = sdpvar(repmat(N_states,1,N_horizon+1),repmat(1,1,N_horizon+1));
u_int = intvar(repmat(N_inputs-1,1,N_horizon), ones(1,N_horizon));
u_var = sdpvar(repmat(1,1,N_horizon), ones(1,N_horizon));
% u = [u_int; u_var]

Constraints = [x{1}==x0_var]; 
Objective = x{end}'*P*x{end};

for k = 1:N_horizon
   Objective = Objective + 0.5*( x{k}'*Q*x{k} + ([int_in_bounds*u_int{k};u_var{k}])'*R*[int_in_bounds*u_int{k};u_var{k}] );
   Constraints = [Constraints, F*x{k} <= state_bounds];%, G*u{k} <= input_bounds
   Constraints = [Constraints, x{k+1} == A_dis*x{k} + B_dis*[int_in_bounds*u_int{k};u_var{k}]];
   Constraints = [Constraints, -1 <= u_int{k} <=1, -0.0020 <= u_var{k} <= 0.0020];
end

Controller = optimizer(Constraints,Objective,[],x0_var,[int_in_bounds*u_int{1};u_var{1}]);

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

x_complete = recover_eight_state(x_save);
x_euler = zeros(3, N_steps);
for k = 1:N_steps
    x_euler(:,k) = quat2eul(x_complete(5:8, k)')';
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
xlabel('Time [sec]')
legend
title('Inputs over time')

% plot states
nexttile
hold on
for i = 1:N_states
    plot(t,x_save(i,:),'DisplayName',"x_"+num2str(i))
end
hold off
xlabel('Time [sec]')
legend
title('States over time')

%% save figure
figfile = fullfile("figures", "N_step="+num2str(N_steps)+"_N_hor="+num2str(N_horizon)+"_P="+num2str(P_gain));
saveas(fig1,figfile);
return % to not run the continue running controller code

%% Continue running controller
% once an controller is established, you can run extra steps without much
% extra calculations. Only need to change the number of extra steps you want.
N_extra_steps = 100;

%error catch to check if sizes still match
if N_steps ~= size(x_save,2), error(['size of N_steps and x/u_save does not match' newline 'fix issue to run Continue running controller']); end

x_save = [x_save, zeros(N_states, N_extra_steps)];
u_save = [u_save, zeros(N_inputs, N_extra_steps)];
x = x_save(:,N_steps);
for i = N_steps + (1:N_extra_steps)
    u = Controller{x};
    u_save(:,i) = u;
    x = A_dis * x + B_dis * u;
    x_save(:,i) = x;
end

N_steps = N_steps + N_extra_steps;
t = 0:dt:dt*(N_steps-1);