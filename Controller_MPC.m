%% Objective Function LQR
fake_input = sdpvar(1,1);
x0_var = sdpvar(N_states,1);
x = [x0_var, sdpvar(N_states,N_horizon)];
u_int = intvar(repmat(N_inputs-1,1,N_horizon), ones(1,N_horizon));
u_var = sdpvar(repmat(1,1,N_horizon), ones(1,N_horizon));
u_tot = sdpvar(N_inputs,N_horizon);

Constraints = [];%[x(:,1)==x0_var]; 
Objective = 0;

for k = 1:N_horizon
   x(:,k+1) = A_dis*x(:,k) + B_dis*[int_in_bounds*u_int{k};u_var{k}];
   u_tot(:,k) = [int_in_bounds*u_int{k};u_var{k}];
   Objective = Objective + 0.5* x(:,k)'*Q*x(:,k) ;
   Objective = Objective + 0.5* u_tot(:,k)'*R*u_tot(:,k);
   Constraints = [Constraints; F*x(:,k+1) <= state_bounds];%, G*u{k} <= input_bounds
   Constraints = [Constraints; -ones(3,1) <= u_int{k} ; u_int{k} <=ones(3,1)];
   Constraints = [Constraints; -2e-3 <= u_var{k}      ; u_var{k} <= 2e-3];
end
Objective = Objective + 0.5*x(:,N_horizon+1)'*P*x(:,N_horizon+1);

options = sdpsettings('solver','mosek','allowmilp',1);
Controller = optimizer(Constraints,Objective,options,{x0_var,fake_input},u_tot(:,1));

%---------------------------
% x0_var = sdpvar(7,1);
% fake_input = sdpvar(1,1);
% x = sdpvar(repmat(N_states,1,N_horizon+1),repmat(1,1,N_horizon+1));
% u_int = intvar(repmat(N_inputs-1,1,N_horizon), ones(1,N_horizon));
% u_var = sdpvar(repmat(1,1,N_horizon), ones(1,N_horizon));
% u_tot = sdpvar(repmat(N_inputs,1,N_horizon), ones(1,N_horizon));
% 
% Constraints = [x{1}==x0_var]; 
% Objective = x{end}'*P*x{end};
% 
% for k = 1:N_horizon
%    u_tot{k} = ([int_in_bounds*u_int{k};u_var{k}]);
%    Objective = Objective + 0.5*( x{k}'*Q*x{k} + u_tot{k}'*R*u_tot{k} );
%    Constraints = [Constraints, F*x{k+1} <= state_bounds];%, G*u{k} <= input_bounds
%    Constraints = [Constraints, x{k+1} == A_dis*x{k} + B_dis*[int_in_bounds*u_int{k};u_var{k}]];
%    Constraints = [Constraints, -1 <= u_int{k} <=1, -0.0020 <= u_var{k} <= 0.0020];
% end
% 
% Controller = optimizer(Constraints,Objective,[],{x0_var,fake_input},u_tot{1});