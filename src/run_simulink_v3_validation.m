function campaign = run_simulink_v3_validation(indices)
% Representative scenarios first; stop on first mismatch, preserving evidence.
if nargin<1,indices=[1 2 11 3:10 12];end
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);scope=project_environment(); %#ok<NASGU>
verify_source_integrity();manifest=struct('version','MPC_v3_integral_reference_hold');checked=0;
outdir=fullfile(root,'results','simulink_v3');modelFile=fullfile(root,'models','auv_v3_closed_loop.slx');
modelHash=auv_file_sha256(modelFile);scenarios=current_design_scenarios();rows=struct([]);
for j=indices
 for mode=1:2
  label='PID';if mode==2,label='MPC_v3';end
  filename=fullfile(outdir,[scenarios{j}.id '_' label '_' num2str(scenarios{j}.duration) 's.mat']);
  result=run_simulink_v3_case(j,mode);
  result.modelSha256=modelHash;save(filename,'result','-v7.3');
  row=result.summary;
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
  campaign=struct('status','in_progress','completedRuns',numel(rows),'allPassed',all([rows.passed]), ...
   'baselineVersion',manifest.version,'frozenSourceFilesChecked',checked,'modelSha256',modelHash,'rows',rows);
  save(fullfile(outdir,'campaign_summary.mat'),'campaign');
  fid=fopen(fullfile(outdir,'campaign_summary.json'),'w');fprintf(fid,'%s',jsonencode(campaign,'PrettyPrint',true));fclose(fid);
  if ~row.passed,error('AUV:CrossValidation','Cross-validation mismatch. Raw trajectory and first mismatch time saved.');end
 end
end
campaign.status='completed';save(fullfile(outdir,'campaign_summary.mat'),'campaign');
fid=fopen(fullfile(outdir,'campaign_summary.json'),'w');c=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(campaign,'PrettyPrint',true));
% Scalar fields only for readable CSV; per-channel errors remain in JSON/MAT.
T=struct2table(rows);T=removevars(T,{'stateErrorByColumn','commandErrorByColumn','maxActualRate'});
writetable(T,fullfile(outdir,'cross_validation_metrics.csv'));
fprintf('SIMULINK CAMPAIGN COMPLETED: %d/%d passed; maximum state difference %.9g; command %.9g.\n', ...
 numel(rows),numel(rows),max([rows.maxStateError]),max([rows.maxCommandError]));
end
