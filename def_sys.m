%% Initial state, input
% state x = [w_1 w_2 w_3 w_w e_1 e_2 e_3]';
% - w_i : angular velocity of the body frame relative to the orbit frame
% - w_w : angular velocity of the wheel relative to the body
% - e_i : Euler parameters (toghether with n)
% - input u = [t_1 t_2 t_3 t_w]';
% - t_i : torque provided from thrusters
% - t_w : internal axial torques applied by the platform to the wheel

global N_inputs
global N_states
global A_dis
global B_dis
global int_in_bounds
global max_state_bounds
global P

N_inputs = 4;
N_states = 7;
N_steps = 0;
x_save = x0;
u_save = [];
x_LQR = x0;
u_LQR = [];
obj_save = [];
obj2_save = [];

%% Continuous state space model
[A_con,B_con] = con_System();
D_con = zeros(N_states,N_inputs);
C_con = eye(N_states); 
%C_con = [eye(4) zeros(4,N_states-4))]; % idea: change C to only measure first 4 physicall states (sys still observable)
sys_con = ss(A_con,B_con,C_con,D_con);

%% Discretise system (less accurate then lsim)
sys_dis = c2d(sys_con, dt, 'zoh');
A_dis = sys_dis.A;
B_dis = sys_dis.B;

%% Constraints definition
% State constraints in the form Fx <= state_bounds
F = [eye(N_states); -eye(N_states)];
max_state_bounds = [1; 1; 1; 800; 1; 1; 1];
state_bounds = [max_state_bounds; max_state_bounds];

% Input constraints
input_bounds = [0.0484; 0.0484; 0.0398; 0.0020];
int_in_bounds = diag(input_bounds(1:3)); 

%% def LQR terms
P = P_gain*eye(N_states);