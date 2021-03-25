N_extra_steps = 200;

x_save = [x_save, zeros(N_states, N_extra_steps)];
u_save = [u_save, zeros(N_inputs, N_extra_steps)];

for i = N_steps + (1:N_extra_steps)
    u = Controller{x_save(:,i)};
    u_save(:,i) = u;
    x_save(:,i+1) = A_dis * x_save(:,i) + B_dis * u;
end

N_steps = N_steps + N_extra_steps;
t = 0:dt:dt*(N_steps-1);