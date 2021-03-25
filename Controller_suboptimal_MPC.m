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
Constraints = [Constraints, Objective_u <= 0.9*Objective_u_tilde];

Controller = optimizer(Constraints,[],[],{x0_var, u_tilde},u_tot);