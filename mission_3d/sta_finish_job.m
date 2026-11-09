function sta_finish_job(file)
% Owner's sequential postprocessing in the same MATLAB session, no simulation.
folder=fileparts(file);
try
 sta_audit_job(file);
 assert(isfile(fullfile(folder,'AUDITS_DONE.txt'))&&~isfile(fullfile(folder,'AUDITS_ERROR.txt')),'Inspect the raw audit error before making qualification claims.');
 sta_export(file,fullfile(folder,'export'));
 sta_export_experiment(file,fullfile(folder,'export'));
 sta_plot_result(file,fullfile(folder,'figures'));
 sta_motion_evidence(file,fullfile(folder,'figures'));
 result=struct('plane',sta_terrain_plane_contract(),'profile',sta_terrain_profile_contract());
 fid=fopen(fullfile(folder,'terrain_contracts_executed.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
 fid=fopen(fullfile(folder,'POSTPROCESS_DONE.txt'),'w');fprintf(fid,'Actual audit, exports, plots and terrain contracts executed in the shared MATLAB.');fclose(fid);
catch err
 fid=fopen(fullfile(folder,'POSTPROCESS_ERROR.txt'),'w');fprintf(fid,'%s',getReport(err,'extended'));fclose(fid);
end
end
