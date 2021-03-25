%% Objective Function LQR
Q = diag([500, 500, 500, 1e-7, 1, 1, 1]);
R = diag([200, 200, 200, 1]);

P = P_gain*eye(N_states);
x0_var = sdpvar(7,1);
x = sdpvar(repmat(N_states,1,N_horizon+1),repmat(1,1,N_horizon+1));
u_int = intvar(repmat(N_inputs-1,1,N_horizon), ones(1,N_horizon));
u_var = sdpvar(repmat(1,1,N_horizon), ones(1,N_horizon));
u_tot = sdpvar(repmat(N_inputs,1,N_horizon), ones(1,N_horizon));

Constraints = [x{1}==x0_var]; 
Objective = x{end}'*P*x{end};

for k = 1:N_horizon
   u_tot{k} = ([int_in_bounds*u_int{k};u_var{k}]);
   Objective = Objective + 0.5*( x{k}'*Q*x{k} + u_tot{k}'*R*u_tot{k} );
   Constraints = [Constraints, F*x{k} <= state_bounds];%, G*u{k} <= input_bounds
   Constraints = [Constraints, x{k+1} == A_dis*x{k} + B_dis*[int_in_bounds*u_int{k};u_var{k}]];
   Constraints = [Constraints, -1 <= u_int{k} <=1, -0.0020 <= u_var{k} <= 0.0020];
end

Controller = optimizer(Constraints,Objective,[],x0_var,u_tot);