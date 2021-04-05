function [u_result,x_result,Objective_result,sol] = Controller_MPC(x0_var)
    global N_horizon
    global N_inputs
    global A_dis
    global B_dis
    global int_in_bounds
    global max_state_bounds
    global R
    global Q
    global P
    %% Objective Function LQR
    x = [x0_var, sdpvar(7,N_horizon)];
    u_int = intvar(repmat(N_inputs-1,1,N_horizon), ones(1,N_horizon));
    u_var = sdpvar(repmat(1,1,N_horizon), ones(1,N_horizon));
    u_tot = sdpvar(N_inputs,N_horizon);

    Constraints = [];
    Objective = 0;

    for k = 1:N_horizon
       x(:,k+1) = A_dis*x(:,k) + B_dis*[int_in_bounds*u_int{k};u_var{k}];
       u_tot(:,k) = [int_in_bounds*u_int{k};u_var{k}];
       Objective = Objective + 0.5* x(:,k)'*Q*x(:,k) ;
       Objective = Objective + 0.5* u_tot(:,k)'*R*u_tot(:,k);
       Constraints = [Constraints; x(:,k+1) <= max_state_bounds; x(:,k+1)>= -max_state_bounds];
       Constraints = [Constraints; -ones(3,1) <= u_int{k} ; u_int{k} <=ones(3,1)];
       Constraints = [Constraints; -2e-3 <= u_var{k}      ; u_var{k} <= 2e-3];
    end
    Objective = Objective + 0.5*x(:,N_horizon+1)'*P*x(:,N_horizon+1);

    options = sdpsettings('solver','mosek','verbose',1,'mosek.MSK_IPAR_SIM_MAX_ITERATIONS',100);
    sol = optimize(Constraints,Objective,options);
    u_result = value(u_tot);
    x_result = value(x);
    Objective_result = value(Objective);
end