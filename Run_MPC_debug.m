x_save = [[x0;1],zeros(N_states+1, N_extra_steps)];
u_save = zeros(N_inputs, N_extra_steps);
u_tilde = zeros(N_inputs, N_horizon); 

for i = (1:N_extra_steps)
    [u_tot,sol,x,objective,objective2] = Controller_suboptimal_MPC_debug(x_save(1:7,i),u_tilde);
    u_save(:,i) = u_tot(:,1);
    u_tilde = [u_tot(:,2:end) zeros(N_inputs,1)];
    x_save(:,i+1) = x_save(:,i) + dt * non_lin_model(x_save(:,i),u_tot(:,1));
end

N_steps = N_steps - N_extra_steps;
t = 0:dt:dt*(N_steps-1);
