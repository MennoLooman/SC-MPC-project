rr_suboptimal_MPC = 0;       % [0/1/2] dont run / run MPC / run suboptimal MPC
rr_solve_DARE = 2;           % [0/1/2] use P=I / DARE -> P / LQR infinite horizon optimal feedback

P_gain = 1;                          % weight of terminal cost
Q = diag([500, 500, 500, 1e-7, 20, 20, 20]);
R = diag([5, 5, 5, 1]);
x0_quat = eul2quat([deg2rad(0.0), deg2rad(-0.0), deg2rad(0.0)], 'XYZ');
x0 = [-0.0 0.0 0.0 0.000 x0_quat(2:4)]';   % initial state
dt = 0.1;                               % sampling rate
N_extra_steps = 400;                    % number of steps each run

rr_non_lin_model = 0;
%% define system
def_sys;
u = [zeros(N_inputs,50), 0.0484*[ones(1,50);zeros(3,50)],zeros(N_inputs,50),-0.0484*[ones(1,50);zeros(3,50)],zeros(N_inputs,50),-0.0484*[zeros(1,50);ones(1,50);zeros(2,50)],zeros(N_inputs,50),0.0484*[zeros(1,50);ones(1,50);zeros(2,50)],zeros(N_inputs,50),0.0398*[zeros(2,50);ones(1,50);zeros(1,50)],zeros(N_inputs,50),-0.0398*[zeros(2,50);ones(1,50);zeros(1,50)],zeros(N_inputs,50),-0.002*[zeros(3,50);ones(1,50)],zeros(N_inputs,50),0.002*[zeros(3,50);ones(1,50)]];
u = [0.0484*[ones(1,50);zeros(3,50)],-0.0484*[ones(1,50);zeros(3,50)],-0.0484*[zeros(1,50);ones(1,50);zeros(2,50)],0.0484*[zeros(1,50);ones(1,50);zeros(2,50)],0.0398*[zeros(2,50);ones(1,50);zeros(1,50)],-0.0398*[zeros(2,50);ones(1,50);zeros(1,50)],-0.002*[zeros(3,50);ones(1,50)],0.002*[zeros(3,50);ones(1,50)]];

x_LQR = [x0 , zeros(N_states,N_extra_steps)];
for i=1:N_extra_steps
    x_LQR(:,i+1) = A_dis * x_LQR(:,i) + B_dis * u(:,i);
end
if length(x_LQR(:,1)) == 7
    x_LQR_8 = recover_eight_state(x_LQR);
elseif length(x_LQR(:,1)) == 8
    x_LQR_8(1:4,:) = x_LQR(1:4,:);
    x_LQR_8(5,:) = x_LQR(8,:);
    x_LQR_8(6:8,:) = x_LQR(5:7,:);
else
    error("error converting state to 8 states");
end
x_LQR_lin = x_LQR_8;

%nonlin
rr_non_lin_model = 1;
eta = sqrt(1 - x0(5:7)'*x0(5:7));
x_LQR = [[x0;eta], zeros(8, N_extra_steps)];
for i=1:N_extra_steps
   x_LQR(:,i+1) = x_LQR(:,i) + dt * non_lin_model(x_LQR(:,i),u(:,i)); 
end

if length(x_LQR(:,1)) == 7
    x_LQR_8 = recover_eight_state(x_LQR);
elseif length(x_LQR(:,1)) == 8
    x_LQR_8(1:4,:) = x_LQR(1:4,:);
    x_LQR_8(5,:) = x_LQR(8,:);
    x_LQR_8(6:8,:) = x_LQR(5:7,:);
else
    error("error converting state to 8 states");
end
x_LQR_nonlin = x_LQR_8;
%% verwerken
t = 0:dt:dt*(N_extra_steps-1);
x_LQR_diff_abs = abs(x_LQR_lin(:,2:end)-x_LQR_nonlin(:,2:end));
x_LQR_diff_relative = x_LQR_diff_abs ./ abs(x_LQR_lin(:,2:end));

figure();
hold on
for i = [1:8]%3 5:7]
    plot(t,x_LQR_diff_abs(i,:),'DisplayName',"x_"+num2str(i),'LineWidth', 2)
end
hold off
%title("Absolute error between nonlinear and linearized dynamics");
grid on
legend('Location','northwest')
xlabel('Time [sec]');
ylabel('Absolute error [-]');

figure();
hold on
for i = [1:7]
    plot(t,x_LQR_diff_relative(i,:),'DisplayName',"x_"+num2str(i))
end
hold off
axis([-inf,inf,0,inf]);
title("Relative error between nonlinear and linearized dynamics");
legend
xlabel('time steps');
ylabel('relative error');

function [state_seq_8] = recover_eight_state(state_seq_7)
%RECOVER_EIGHT_STATE Returns the complete state description based on the
%states comming from the MPC
%   only works when simulation time is > 7
%   the extra state is inserted in the place as mentioned to the article.
    N = length(state_seq_7);
    state_seq_8 = zeros(8, N);
    state_seq_8(1:4, :) = state_seq_7(1:4, :);
    state_seq_8(6:8, :) = state_seq_7(5:7, :);
    
    for i = 1:N
        eta = sqrt(1 - state_seq_8(6:8, i)'*state_seq_8(6:8, i));
        state_seq_8(5, i) = eta;
    end
end