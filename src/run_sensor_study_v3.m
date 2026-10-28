function rows = run_sensor_study_v3(indices)
if nargin<1,indices=[1 2 11];end
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);scope=project_environment(); %#ok<NASGU>
out=fullfile(root,'results','sensor_study_v3');if ~isfolder(out),mkdir(out);end;cfg=current_control_parameters();cases=current_design_scenarios();trim=compute_trim();rows=struct([]);
for index=indices
 sc=cases{index};
 for condition={'delay','noise','noise_delay'}
  seeds=104729;if ~strcmp(condition{1},'delay'),seeds=[104729 130363 155921];end
  for seed=seeds
   sensor=sensor_study_configuration(condition{1},seed);
   for mode=1:2
    M=[];label='PID';if mode==2,M=cfg.M;label='MPC_v3';end
    tag=[sc.id '_' condition{1} '_' num2str(seed) '_' label];file=fullfile(out,[tag '.mat']);
    if isfile(file),a=load(file,'run');run=a.run;assert(isequal(run.sensor,sensor));
    else
     run=simulate_sensor_control(cfg.C,trim,sc,cfg.h,M,sensor);run.controller=label;run.version=cfg.version;save(file,'run','-v7.3');
    end
    m=run.metrics;steady=run.t>=run.t(end)-50;fail=0;violation=0;retry=0;
    if mode==2,fail=run.solverMetrics.failureCount;violation=run.solverMetrics.maxViolation;retry=run.solverMetrics.retryCount;end
    row=struct('caseId',sc.id,'condition',condition{1},'seed',seed,'method',label,'status',run.status,'duration',run.t(end), ...
     'rmsePost30',m.rmsePost30,'steadyRmse',sqrt(mean(run.ey(steady).^2)),'rudderTVrad',m.rudderTVrad, ...
     'peakPost30',m.peakPost30,'rudderPeakDeg',m.maxRudderDeg,'commandRateDegS',rad2deg(m.maxCommandRate(1)), ...
     'speedSteadyMaxError',m.maxSpeedErrorSteady,'depthSteadyMaxError',m.maxDepthErrorSteady, ...
     'finalQpFailures',fail,'sameQpRetries',retry,'inputViolation',violation);
    if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
    fprintf('SENSOR %s %s seed%d %s: %s RMSE %.7g TV %.7g failures%d\n',sc.id,condition{1},seed,label,run.status,m.rmsePost30,m.rudderTVrad,fail);
   end
  end
 end
end
writetable(struct2table(rows),fullfile(out,['metrics_cases_' char(strjoin(string(indices),'_')) '.csv']));
end
