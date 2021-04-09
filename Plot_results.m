%% plot results
t = 0:dt:dt*(N_steps-1);

if length(x_save(:,1)) == 7
    x_save_8 = recover_eight_state(x_save);
elseif length(x_save(:,1)) == 8
    x_save_8 = x_save;
else
    error("error converting state to 8 states");
end

% convert quaternions to euler angles.
euler = zeros(3, length(x_save_8));
for i =1:length(x_save_8)
    euler(:,i) = rad2deg(quat2eul(x_save_8(5:8, i)'));
end

save(strcat('Simulation_result_N_', num2str(N_horizon),'_suboptimal'), 'euler', 'obj_save', 'x_save', 'u_save', 'Q', 'R');

figure(10); hold on; grid on; % Plot euler angles
plot(t, euler(1,2:end), 'DisplayName','yaw', 'LineWidth', 2);
plot(t, euler(2,2:end), 'DisplayName','pitch', 'LineWidth', 2);
plot(t, euler(3,2:end), 'DisplayName','roll', 'LineWidth', 2);
legend(); title('Orientation of satellite in Euler angles');
xlabel('Time [sec]'); ylabel('Angle [deg]');

if rr_suboptimal_MPC
    fig1 = figure();
    til1 = tiledlayout(2,2);

    % plot inputs
    nexttile
    hold on
    for i = 1:N_inputs-1
        plot(t,u_save(i,:),'DisplayName',"u_"+num2str(i))
    end
    hold off
    axis([-inf inf -5e-2 5e-2]);
    xlabel('Time [sec]')
    legend
    title('Inputs over time')

    nexttile
    plot(t,u_save(4,:),'DisplayName',"u_4")
    legend
    axis([-inf inf -2e-3 2e-3]);
    xlabel('Time [sec]')

    % plot states
    %note that the state is: x=[w_1 w_2 w_3 w_w e_1 e_2 e_3, n]' with n last
    nexttile
    hold on
    for i = [1:3 5:7]
        plot(t,x_save(i,2:end),'DisplayName',"x_"+num2str(i))
    end
    hold off
    axis([-inf inf -1 1])
    xlabel('Time [sec]')
    legend
    title('States over time')

    nexttile
    if rr_non_lin_model, plot_range = [4,8]; else, plot_range = 4; end
    hold on
    for i = plot_range
        plot(t,x_save(i,2:end),'DisplayName',"x_"+num2str(i))
    end
    hold off
    xlabel('Time [sec]')
    legend
    title('States over time')
    
    fig4 = figure();
    if rr_suboptimal_MPC==2
        title_MPC = "Suboptimal MPC "; 
        plot(t,obj_save,'DisplayName',"New found cost");
        hold on 
        plot(t,obj2_save,'DisplayName',"Cost using u_tilde");
        hold off
    else
        plot(t,obj_save,'DisplayName',"Optimal cost");
        title_MPC = "Finite Horizon MPC "; 
    end
    xlabel('Time [sec]')
    legend
    title("Objective Cost funtion V_N(x,u) of "+title_MPC);
    
    if rr_non_lin_model, title_lin = "with nonlinear model"; else, title_lin = "with linearized model"; end
    title(til1,title_MPC+title_lin)
end

%% plot results of LQR optimal control
if rr_solve_DARE==2
    fig3 = figure();
    til3 = tiledlayout(2,2);

    % plot inputs
    nexttile
    hold on
    for i = 1:N_inputs-1
        plot(t,u_LQR(i,:),'DisplayName',"u_"+num2str(i))
    end
    hold off
    axis([-inf inf -5e-2 5e-2]);
    xlabel('Time [sec]')
    legend
    title('Inputs over time')

    nexttile
    plot(t,u_LQR(4,:),'DisplayName',"u_4")
    legend
    axis([-inf inf -2e-3 2e-3]);
    xlabel('Time [sec]')

    % plot states
    %note that the state is: x=[w_1 w_2 w_3 w_w e_1 e_2 e_3, n]' with n last
    nexttile
    hold on
    for i = [1:3 5:7]
        plot(t,x_LQR(i,2:end),'DisplayName',"x_"+num2str(i))
    end
    hold off
    axis([-inf inf -1 1])
    xlabel('Time [sec]')
    legend
    title('States over time')

    nexttile
    if rr_non_lin_model, plot_range = [4,8]; else, plot_range = 4; end
    hold on
    for i = plot_range
        plot(t,x_LQR(i,2:end),'DisplayName',"x_"+num2str(i))
    end
    hold off
    xlabel('Time [sec]')
    legend
    title('States over time')

    title(til3,"LQR optimal control")
end

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