function sta_gui_export(file,out)
% Dispatch uses the archived environment, never a current GUI dropdown.
s=load(file,'config');
if strcmp(s.config.execution.environment,'static_unknown')
 sta_export(file,out);sta_export_experiment(file,out);
 sta_plot_result(file,fullfile(out,'figures'));sta_motion_evidence(file,fullfile(out,'figures'));
else,auv_task_export(file,out);end
end
