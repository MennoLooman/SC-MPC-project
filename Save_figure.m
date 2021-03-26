%% save figure
figfile = fullfile("figures", "N_step="+num2str(N_steps)+"_N_hor="+num2str(N_horizon)+"_P="+num2str(P_gain));
if ~isfile(figfile+".fig")
    saveas(fig1,figfile);
else
    error('Save_figures: figure already exists');
end