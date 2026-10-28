function summary = run_design_comparison(indices,recompute)
% Active v3 versus fixed PID. Old v2 numerical records are historical only.
if nargin<1,indices=1:12;end
if nargin<2,recompute=false;end
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);scope=project_environment(); %#ok<NASGU>
out=fullfile(root,'results','final_comparison_v3');if ~isfolder(out),mkdir(out);end
cfg=current_control_parameters();scenarios=current_design_scenarios();trim=compute_trim();config=cfg;
save(fullfile(out,'active_configuration.mat'),'config','trim');signature=jsonencode(cfg);
for j=indices
 sc=scenarios{j};scenarioSignature=jsonencode(rmfield(sc,'force'));
 for m=1:2
  label='PID';if m==2,label='MPC_v3';end
  file=fullfile(out,[sc.id '_' label '.mat']);
  if isfile(file) && ~recompute
   a=load(file,'run');assert(strcmp(a.run.configurationSignature,signature) && strcmp(a.run.scenarioSignature,scenarioSignature));
   assert(strcmp(a.run.status,'completed'));fprintf('Cached final %s %s\n',sc.id,label);continue;
  end
  if m==1,run=simulate_active_control(cfg.C,trim,sc,cfg.h);
  else,run=simulate_active_control(cfg.C,trim,sc,cfg.h,cfg.M);end
  run.controller=label;run.activeVersion=cfg.version;run.configurationSignature=signature;run.scenarioSignature=scenarioSignature;
  run.evaluationLimit=2.5;if endsWith(sc.id,'_NOM'),run.evaluationLimit=1;end
  run.engineeringPass=strcmp(run.status,'completed') && run.metrics.rmsePost30<=run.evaluationLimit ...
   && run.metrics.maxSpeedErrorSteady<=.15 && run.metrics.maxDepthErrorSteady<=.5;
  expectedFile=fullfile(root,'reference_data','v3',[sc.id '_' label '.mat']);
  a=load(expectedFile,'reference');assert(isequal(size(run.z),size(a.reference.z)));
  run.promotionRegressionDifference=max(abs(run.z-a.reference.z),[],'all');
  run.promotionCommandDifference=max(abs(run.command-a.reference.command),[],'all');
  save(file,'run','-v7.3');
  assert(run.promotionRegressionDifference<1e-8 && run.promotionCommandDifference<1e-8, ...
   'AUV:Reference','Computed trajectory differs from bundled numeric reference.');
  fprintf('V3 %s %s: %s RMSE %.8g TV %.8g regression %.3g\n', ...
   sc.id,label,run.status,run.metrics.rmsePost30,run.metrics.rudderTVrad,run.promotionRegressionDifference);
 end
end
complete=true;for j=1:12,for label={'PID','MPC_v3'},complete=complete && isfile(fullfile(out,[scenarios{j}.id '_' label{1} '.mat']));end,end
if complete,summary=summarize_final_comparison();else,summary=struct('status','in_progress');end
end
