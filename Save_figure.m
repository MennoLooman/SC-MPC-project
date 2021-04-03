%% save figure
figfile1 = fullfile("figures", "N_step="+num2str(N_steps)+"_N_hor="+num2str(N_horizon)+"_P="+num2str(P_gain));
if ~isfile(figfile1+".fig")
    saveas(fig1,figfile1);
else
    error('Save_figures: figure already exists');
end

if rr_plot_non_lin
    figfile2 = fullfile("figures", "N_step="+num2str(N_steps)+"_N_hor="+num2str(N_horizon)+"_P="+num2str(P_gain));
    if ~isfile(figfile2+".fig")
        saveas(fig2,figfile2);
    else
        error('Save_figures: figure already exists');
    end
end