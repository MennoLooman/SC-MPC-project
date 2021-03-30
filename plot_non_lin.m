x_save2 = [0.1 0.0 0.0 0.0 0.0 0.0 0.0 1]';
x_save2 = [x_save2, zeros(N_states+1, N_extra_steps)];
for i = (1:N_extra_steps)
    x_save2(:,i+1) = x_save2(:,i) + dt * non_lin_model(x_save2(:,i),u_save(:,i)); %euler forward method, only first order approximation
end
fig2 = figure();
til2 = tiledlayout(2,2);

% plot inputs
nexttile
hold on
for i = 1:N_inputs-1
    plot(t,u_save(i,:),'DisplayName',"u_"+num2str(i))
end
hold off
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
%nexttile(3,[1,2])
nexttile
hold on
for i = [1:3 5:7]
    plot(t,x_save2(i,2:end),'DisplayName',"x_"+num2str(i))
end
hold off
axis([-inf inf -1 1])
xlabel('Time [sec]')
legend
title('States over time')

nexttile
hold on
for i = [4,8]
    plot(t,x_save2(i,2:end),'DisplayName',"x_"+num2str(i))
end
hold off
xlabel('Time [sec]')
legend
title('States over time')

if rr_suboptimal_MPC, title_MPC = "Suboptimal MPC "; else, title_MPC = "Finite Horizon MPC "; end
title(til2,title_MPC+"with linearized model, applied on nonlinear model")
disp("max error between nonlinear and linear model: " + max(max(abs(x_save-x_save2(1:7,:)))));