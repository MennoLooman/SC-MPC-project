N_states = 8;
if size(x_save) == [7 1]
    %adding euler parameter n
    x_save = [x_save;1]; 
end
if length(u_save) <= N_horizon
    %this initial input needs to be valid for first N_horizon steps!
    u_tilde = zeros(N_inputs, N_horizon); 
else
    %teaking last N_horizon inputs as 
    u_tilde = [u_save(:,end-N_horizon+1:end) zeros(N_inputs,1)];
end
x_save = [x_save, zeros(N_states, N_extra_steps)];
u_save = [u_save, zeros(N_inputs, N_extra_steps)];

for i = N_steps + (1:N_extra_steps)
    u = cell2mat(Controller{x_save(1:7,i),u_tilde});
    u_save(:,i) = u(:,1);
    u_tilde = [u(:,2:N_horizon) zeros(N_inputs,1)];
    x_save(:,i+1) = x_save(:,i) + dt * non_lin_model(x,u(:,1)); %euler forward method, only first order approximation
end

N_steps = N_steps + N_extra_steps;
t = 0:dt:dt*(N_steps-1);