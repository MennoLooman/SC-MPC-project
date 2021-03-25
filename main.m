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
u_tilde = sdpvar(N_inputs,N_horizon);
x = sdpvar(repmat(N_states,1,N_horizon+1),repmat(1,1,N_horizon+1));
x_tilde = sdpvar(repmat(N_states,1,N_horizon+1),repmat(1,1,N_horizon+1));
u_int = intvar(repmat(N_inputs-1,1,N_horizon), ones(1,N_horizon));
u_var = sdpvar(repmat(1,1,N_horizon), ones(1,N_horizon));
u_tot = sdpvar(repmat(N_inputs,1,N_horizon), ones(1,N_horizon));

Constraints = [x{1}==x0_var]; 
Constraints = [x_tilde{1}==x0_var];
Objective_u = x{end}'*P*x{end};
Objective_u_tilde = x_tilde{end}'*P*x_tilde{end};

for k = 1:N_horizon
   u_tot{k} = ([int_in_bounds*u_int{k};u_var{k}]);
   Objective_u = Objective_u + 0.5*( x{k}'*Q*x{k} + u_tot{k}'*R*u_tot{k} );
   Objective_u_tilde = Objective_u_tilde + 0.5*( x_tilde{k}'*Q*x_tilde{k} + u_tilde(:,k)'*R*u_tilde(:,k) );
   Constraints = [Constraints, F*x{k} <= state_bounds];%, G*u{k} <= input_bounds
   Constraints = [Constraints, x{k+1} == A_dis*x{k} + B_dis*u_tot{k}];
   Constraints = [Constraints, x_tilde{k+1} == A_dis*x_tilde{k} + B_dis*u_tilde(:,k)];
   Constraints = [Constraints, -1 <= u_int{k} <=1, -0.0020 <= u_var{k} <= 0.0020];
end
Constraints = [Constraints, Objective_u <= Objective_u_tilde];%0.999*

Controller = optimizer(Constraints,[],[],{x0_var, u_tilde},u_tot);

%% Run Controller

N_states = N_states+1;
x_save = zeros(N_states, N_steps);
u_save = zeros(N_inputs, N_steps);
u_tilde = zeros(N_inputs, N_horizon); %this initial input needs to be valid for first N_horizon steps!
x = [x0; 1];

for i = 1:N_steps
    u = cell2mat(Controller{x(1:7),u_tilde});
    u_save(:,i) = u(:,1);
    u_tilde = [u(:,2:N_horizon) zeros(N_inputs,1)];
    x = x + dt * non_lin_model(x,u(:,1)); %euler forward method, only first order approximation
    x_save(:,i) = x;
end

% x_complete = recover_eight_state(x_save);
% x_euler = zeros(3, N_steps);
% for k = 1:N_steps
%     x_euler(:,k) = quat2eul(x_complete(5:8, k)')';
% end

%% plot results
fig1 = figure();
tiledlayout(2,2)

% plot inputs
nexttile
hold on
for i = 1:N_inputs-1
    plot(t,u_save(i,:),'DisplayName',"u_"+num2str(i))
end
hold off
xlabel('Time [sec]')
legend
title('Inputs over time')

nexttile
plot(t,u_save(4,:),'DisplayName',"u_4")
legend
axis([-inf inf -2e-3 2e-3]);
xlabel('Time [sec]')

% plot states
%note that the state is: x=[w_1 w_2 w_3 w_w e_1 e_2 e_3, n]' with n last
%nexttile(3,[1,2])
nexttile
hold on
for i = [1:3 5:7]
    plot(t,x_save(i,:),'DisplayName',"x_"+num2str(i))
end
hold off
axis([-inf inf -1 1])
xlabel('Time [sec]')
legend
title('States over time')

nexttile
hold on
for i = [4,8]
    plot(t,x_save(i,:),'DisplayName',"x_"+num2str(i))
end
hold off
xlabel('Time [sec]')
legend
title('States over time')

return % to not run the save figure code

%% save figure
figfile = fullfile("figures", "N_step="+num2str(N_steps)+"_N_hor="+num2str(N_horizon)+"_P="+num2str(P_gain));
if ~isfile(figfile+".fig")
    saveas(fig1,figfile);
end

return % to not run the continue running controller code

%% Continue running controller
% once an controller is established, you can run extra steps without much
% extra calculations. Only need to change the number of extra steps you want.
tic
N_extra_steps = 2500;

%error catch to check if sizes still match
if N_steps ~= size(x_save,2), error(['size of N_steps and x/u_save does not match' newline 'fix issue to run Continue running controller']); end

x_save = [x_save, zeros(N_states, N_extra_steps)];
u_save = [u_save, zeros(N_inputs, N_extra_steps)];
x = x_save(:,N_steps);
for i = N_steps + (1:N_extra_steps)
    u = Controller{x(1:7)};
    u_save(:,i) = u;
    x = x + dt * non_lin_model(x,u); %euler forward method, only first order approximation
    x_save(:,i) = x;
end

N_steps = N_steps + N_extra_steps;
t = 0:dt:dt*(N_steps-1);
time_run = toc;