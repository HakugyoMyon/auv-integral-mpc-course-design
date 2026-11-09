function result=auv_task_export(file,outdir)
% Human-readable outputs from a completed raw MIL record; no control input.
assert(isfile(file),'Raw MIL file missing.');assert(~isfolder(outdir),'Use a new export folder.');
[~,ram]=memory;assert(ram.PhysicalMemory.Available/2^30>1.25,'Insufficient memory for export.');
s=load(file,'h','plan','config','report','payloadLedger','payloadLog','payloadConfig');mkdir(outdir);
auv_task_save(s.config,fullfile(outdir,'mission.json'));
fid=fopen(fullfile(outdir,'summary.json'),'w');fprintf(fid,'%s',jsonencode(s.report,PrettyPrint=true));fclose(fid);
available=whos('-file',file);
if any(strcmp({available.name},'interactionLog'))
 interaction=load(file,'interactionLog');fid=fopen(fullfile(outdir,'operator_commands.json'),'w');fprintf(fid,'%s',jsonencode(interaction.interactionLog,PrettyPrint=true));fclose(fid);
end
if any(strcmp({available.name},'routeLog'))
 routes=load(file,'routeLog');fid=fopen(fullfile(outdir,'route_changes.json'),'w');fprintf(fid,'%s',jsonencode(routes.routeLog,PrettyPrint=true));fclose(fid);
end
h=s.h;
track=[h.t,h.estimate(:,1:3),h.truth(:,1:3),h.reference(:,1),h.quality(:,1),rad2deg(h.quality(:,2)),h.quality(:,3)];
names={'time_s','north_est_m','east_est_m','depth_est_m','north_truth_m','east_truth_m','depth_truth_m','depth_reference_m','nav_radius95_m','yaw_std_deg','dvl_age_s'};
writetable(array2table(track,'VariableNames',names),fullfile(outdir,'trajectory.csv'));
act=[h.t,h.command(:,1),rad2deg(h.command(:,2:5)),h.estimate(:,14),rad2deg(h.estimate(:,15:18)),h.truth(:,14),rad2deg(h.truth(:,15:18))];
cmd=arrayfun(@(k)sprintf('fin%d_command_deg',k),1:4,'UniformOutput',false);
model=arrayfun(@(k)sprintf('fin%d_model_deg',k),1:4,'UniformOutput',false);
truth=arrayfun(@(k)sprintf('fin%d_truth_deg',k),1:4,'UniformOutput',false);
names=[{'time_s','thrust_command_N'},cmd,{'thrust_model_N'},model,{'thrust_truth_N'},truth];
writetable(array2table(act,'VariableNames',names),fullfile(outdir,'actuators.csv'));
if auv_task_is_point_task(s.plan.task.type),positions=s.plan.targets.positionsNED;
elseif strcmp(s.plan.task.type,'survey'),positions=[s.plan.targets.centersNE,nan(size(s.plan.targets.centersNE,1),1)];
else,positions=[s.plan.targets.positionsNE,nan(size(s.plan.targets.positionsNE,1),1)];end
[trueCounts,falseClaims]=auv_task_truth_counts(s.plan,s.payloadLedger,h,s.payloadLog,s.payloadConfig);
counts=s.payloadLedger.counts;n=numel(counts);data=[(1:n)',positions,counts,repmat(s.payloadLedger.minimumObservations,n,1),trueCounts];
writetable(array2table(data,'VariableNames',{'original_target_id','north_m','east_m','depth_m','accepted_count','required_count','offline_truth_support_count'}),fullfile(outdir,'targets.csv'));
result=struct('exported',true,'controlRows',numel(h.t),'targets',n,'falseConfirmations',falseClaims,'sourceRawMAT',file,...
 'scope','Truth columns are simulator-only offline evidence, never navigation/controller input. Coverage/inspection use geometric sensing, not real imagery. Export does not qualify a failed or unaudited task.');
fid=fopen(fullfile(outdir,'export_manifest.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
