%% plot results
t = 0:dt:dt*(N_steps-1);
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