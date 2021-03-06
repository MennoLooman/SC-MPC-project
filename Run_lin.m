%% run MPC controller
if flag_MPC_type
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
            [u,x,obj,obj2,Vf,lN] = Controller_suboptimal_MPC(x_save(:,i),u_tilde); 
            u_tilde(:,1:N_horizon-1) = u(:,2:end); 
%             u_tilde(:,end) = K_LQR*x(:,end); %last column is optimal control with feasible input
%             for j = 1:3
%                 if u_tilde(j,end) > input_bounds(j)/3, u_tilde(j,end) = input_bounds(j); 
%                 elseif u_tilde(j,end) < -input_bounds(j)/3, u_tilde(j,end) = -input_bounds(j);
%                 else, u_tilde(j,end)=0; 
%                 end
%            end
%            if u_tilde(4,end) > input_bounds(4), u_tilde(4,end) = input_bounds(4);
%            elseif u_tilde(4,end) < -input_bounds(4), u_tilde(4,end) = -input_bounds(4);
%            end
            obj2_save(i) = obj2;
        else
            [u,x,obj,Vf,lN] = Controller_MPC(x_save(:,i)); 
        end 
        Vf_save(i) = Vf;
        lN_save(i) = lN;
        obj_save(i) = obj;
        u_save(:,i) = u(:,1);
        x_save(:,i+1) = A_dis * x_save(:,i) + B_dis * u(:,1);
    end
    
    %time estimate single run
    if flag_MPC_type == 2
        tic
        Controller_suboptimal_MPC(x_save(:,1),zeros(N_inputs, N_horizon)); 
        time_sub_MPC = toc;
        tic
        Controller_suboptimal_MPC(x_save(:,i+1),u_tilde); 
        time_sub_MPC = [time_sub_MPC, toc];
    else
        tic
        Controller_MPC(x_save(:,1)); 
        time_MPC = toc;
        tic
        Controller_MPC(x_save(:,i+1)); 
        time_MPC = [time_MPC, toc];
    end
end

%% run system with optimal LQR feedback
if flag_P_type==2
    x_LQR = [x_LQR, zeros(N_states, N_extra_steps)];
    u_LQR = [u_LQR, zeros(N_inputs, N_extra_steps)];
    for i = N_steps + (1:N_extra_steps)
       u_LQR(:,i) = -K_LQR * x_LQR(:,i);
       %adjusting input to input constraints
       for j = 1:3
            if u_LQR(j,i) > input_bounds(j)/3, u_LQR(j,i) = input_bounds(j); 
            elseif u_LQR(j,i) < -input_bounds(j)/3, u_LQR(j,i) = -input_bounds(j);
            else, u_LQR(j,i)=0; 
            end
       end
       if u_LQR(4,i) > input_bounds(4), u_LQR(4,i) = input_bounds(4);
       elseif u_LQR(4,i) < -input_bounds(4), u_LQR(4,i) = -input_bounds(4);
       end
       %time step
       x_LQR(:,i+1) = A_dis * x_LQR(:,i) + B_dis * u_LQR(:,i); 
    end
end

N_steps = N_steps + N_extra_steps;