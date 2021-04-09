N_states = 8;
%% run MPC controller
if size(x_save) == [7,1]
    %adding initial value for euler parameter n
    eta = sqrt(1 - x_save(5:7)'*x_save(5:7));
    x_save = [x_save;eta]; 
    x_LQR = [x_LQR;eta];
end
if flag_MPC_type
    %nonlinear model uses the extra (uncontrolable, unobservable) euler parameter
    %the suboptimal MPC uses the warm start u_tilde to have a reference input
    if flag_MPC_type == 2
        obj2_save = [obj2_save, zeros(1,N_extra_steps)];
        if length(u_save) <= N_horizon
            %this initial input needs to be valid for first N_horizon steps!
            u_tilde = zeros(N_inputs, N_horizon); 
        else
            %teaking last inputs as warm start, note last column is zero
            u_tilde(:,1:N_horizon-1) = u_save(:,end-N_horizon+2:end);
        end
    end

    x_save = [x_save, zeros(N_states, N_extra_steps)];
    u_save = [u_save, zeros(N_inputs, N_extra_steps)];
    obj_save = [obj_save, zeros(1,N_extra_steps)];
    Vf_save = [Vf_save, zeros(1, N_extra_steps)];
    lN_save = [lN_save, zeros(1, N_extra_steps)];

    for i = N_steps + (1:N_extra_steps)
        if flag_MPC_type == 2
            [u,x,obj,obj2,Vf,lN] = Controller_suboptimal_MPC(x_save(1:N_states-1,i),u_tilde); 
            u_tilde(:,1:N_horizon-1) = u(:,2:end); %last column of u_tilde is always zero
            obj2_save(i) = obj2;
        else
            [u,x,obj,Vf,lN] = Controller_MPC(x_save(1:N_states-1,i)); 
        end 
        Vf_save(i) = Vf;
        lN_save(i) = lN;
        obj_save(i) = obj;
        u_save(:,i) = u(:,1);
        %euler forward method, only first order approximation
        x_save(:,i+1) = x_save(:,i) + dt * non_lin_model(x_save(:,i),u(:,1));
    end
    
    %time estimate single run
    if flag_MPC_type == 2
        tic
        Controller_suboptimal_MPC(x_save(1:7,1),zeros(N_inputs, N_horizon)); 
        time_sub_MPC = toc;
        tic
        Controller_suboptimal_MPC(x_save(1:7,i+1),u_tilde); 
        time_sub_MPC = [time_sub_MPC, toc];
    else
        tic
        Controller_MPC(x_save(1:7,1)); 
        time_MPC = toc;
        tic
        Controller_MPC(x_save(1:7,i+1)); 
        time_MPC = [time_MPC, toc];
    end
end

%% run system with optimal LQR feedback
if flag_P_type==2
    x_LQR = [x_LQR, zeros(N_states, N_extra_steps)];
    u_LQR = [u_LQR, zeros(N_inputs, N_extra_steps)];
    for i = N_steps + (1:N_extra_steps)
       u_LQR(:,i) = [-K_LQR,zeros(N_inputs,1)] * x_LQR(:,i);
       for j = 1:3
            if u_LQR(j,i) > input_bounds(j)/3, u_LQR(j,i) = input_bounds(j); 
            elseif u_LQR(j,i) < -input_bounds(j)/3, u_LQR(j,i) = -input_bounds(j);
            else, u_LQR(j,i)=0; 
            end
       end
       if u_LQR(4,i) > input_bounds(4), u_LQR(4,i) = input_bounds(4);
       elseif u_LQR(4,i) < -input_bounds(4), u_LQR(4,i) = -input_bounds(4);
       end
       x_LQR(:,i+1) = x_LQR(:,i) + dt * non_lin_model(x_LQR(:,i),u_LQR(:,i));
    end
end

N_steps = N_steps + N_extra_steps;