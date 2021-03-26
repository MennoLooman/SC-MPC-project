%% plot results
fig1 = figure();
tiledlayout(2,2)

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