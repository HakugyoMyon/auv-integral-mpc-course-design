function summary = summarize_final_comparison()
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'results','final_comparison_v3');
config=current_control_parameters();scenarios=current_design_scenarios();rows=struct([]);pairs=struct([]);
for j=1:numel(scenarios)
 scenario=scenarios{j};results=cell(2,1);
 for m=1:2
  label='PID';if m==2,label='MPC_v3';end
  saved=load(fullfile(out,[scenario.id '_' label '.mat']),'run');rr=saved.run;results{m}=rr;
  mm=rr.metrics;solverFailures=NaN;solve95=NaN;total95=NaN;violation=NaN;
  if m==2
   solverFailures=rr.solverMetrics.failureCount;solve95=rr.solverMetrics.p95_s;
   total95=rr.solverMetrics.modelQpTotalP95_s;violation=rr.solverMetrics.maxViolation;
  end
  limit=2.5;isNominal=endsWith(scenario.id,'_NOM');if isNominal,limit=1;end
  pass=strcmp(rr.status,'completed') && mm.rmsePost30<=limit && mm.maxSpeedErrorSteady<=.15 && mm.maxDepthErrorSteady<=.5;
  if isNominal,pass=pass && mm.peakPost30<=2;end
  if strcmp(scenario.id,'LINE_NOM'),pass=pass && mm.settlingTime<=60;end
  primaryFailures=0;retryCount=0;retryRecovered=0;
  if m==2 && isfield(rr.solverMetrics,'primaryFailureCount')
   primaryFailures=rr.solverMetrics.primaryFailureCount;retryCount=rr.solverMetrics.retryCount;retryRecovered=rr.solverMetrics.retryRecoveredCount;
  end
  reg=NaN;if isfield(rr,'promotionRegressionDifference'),reg=rr.promotionRegressionDifference;end
  methodVersion=config.pidVersion;if m==2,methodVersion=config.version;end
  row=struct('caseId',scenario.id,'path',scenario.path,'method',label,'version',methodVersion, ...
   'status',rr.status,'failure',rr.failure,'actualDuration',rr.t(end),'rmseAll',mm.rmseAll, ...
   'rmsePost30',mm.rmsePost30,'peakPost30',mm.peakPost30,'steadyBias',mm.steadyBias, ...
   'steadyPeak',mm.steadyPeak,'headingRmseDeg',mm.headingRmseDeg, ...
   'steadyInnerMeanDeg',rr.diagnostics.steadyInnerMeanDeg,'rudderTVrad',mm.rudderTVrad, ...
   'rudderPeakDeg',mm.maxRudderDeg,'maxCommandRateDegS',rad2deg(mm.maxCommandRate(1)), ...
   'maxActualRateDegS',rad2deg(mm.maxActualRate(1)),'speedSteadyMaxError',mm.maxSpeedErrorSteady, ...
   'depthSteadyMaxError',mm.maxDepthErrorSteady,'etaPeakDegS',rr.diagnostics.etaPeakDegS, ...
   'etaFreezeFraction',rr.diagnostics.etaFreezeFraction,'solverFailures',solverFailures, ...
   'primaryQpFailures',primaryFailures,'sameQpRetries',retryCount,'retryRecoveries',retryRecovered,'solverP95_s',solve95,'modelQpP95_s',total95,'inputViolation',violation, ...
   'engineeringTargetPass',pass,'evaluationRmseLimit',limit,'promotionRegressionDifference',reg);
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW> Exactly 24 rows.
 end
 p=results{1};q=results{2};
 pair=struct('caseId',scenario.id,'pidStatus',p.status,'mpcStatus',q.status, ...
  'pidRmse',p.metrics.rmsePost30,'mpcRmse',q.metrics.rmsePost30, ...
  'deltaRmse_m',q.metrics.rmsePost30-p.metrics.rmsePost30, ...
  'pidTVrad',p.metrics.rudderTVrad,'mpcTVrad',q.metrics.rudderTVrad, ...
  'pidPeakRudderDeg',p.metrics.maxRudderDeg,'mpcPeakRudderDeg',q.metrics.maxRudderDeg);
 if isempty(pairs),pairs=pair;else,pairs(end+1)=pair;end %#ok<AGROW> Exactly 12 pairs.
end
T=struct2table(rows);P=struct2table(pairs);
writetable(T,fullfile(out,'final_metrics.csv'));writetable(P,fullfile(out,'pid_mpc_v3_pairs.csv'));
summary=struct('activeVersion',config.version,'retiredMpcIncluded',false,'scenarioCount',numel(scenarios), ...
 'runCount',height(T),'allCompleted',all(strcmp(T.status,'completed')), ...
 'engineeringPassed',sum(T.engineeringTargetPass),'solverFailures',sum(T.solverFailures,'omitnan'), ...
 'primaryQpFailures',sum(T.primaryQpFailures),'sameQpRetries',sum(T.sameQpRetries),'retryRecoveries',sum(T.retryRecoveries),'maxInputViolation',max(T.inputViolation,[],'omitnan'),'metrics',rows,'pairs',pairs, ...
 'scope','Integral MPC v3 versus frozen PID; MATLAB campaign only, no Simulink verification.');
save(fullfile(out,'final_comparison_summary.mat'),'summary','T','P');
fid=fopen(fullfile(out,'final_comparison_summary.json'),'w');assert(fid>=0);fileCleanup=onCleanup(@() fclose(fid));
fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));
fig=figure('Name','Current PID and updated MPC campaign','Color','w','Position',[50 50 1250 820]);tiledlayout(2,2);
nexttile;bar([P.pidRmse P.mpcRmse]);xticks(1:height(P));xticklabels(P.caseId);xtickangle(45);grid on;
ylabel('Post-30s cross-track RMSE (m)');legend('PID','Integral MPC v3');
nexttile;bar([P.pidTVrad P.mpcTVrad]);xticks(1:height(P));xticklabels(P.caseId);xtickangle(45);grid on;
ylabel('Actual rudder total variation (rad)');legend('PID','Integral MPC v3');
for c=1:2
 nexttile;id={'LINE_NOM','CIRCLE_NOM'};
 for m=1:2
  method={'PID','MPC_v3'};rr=load(fullfile(out,[id{c} '_' method{m} '.mat']),'run');hold on;
  plot(rr.run.t,rr.run.ey,'LineWidth',1.2);
 end
 grid on;xlabel('Time (s)');ylabel('Cross-track error (m)');title(strrep(id{c},'_',' '));legend('PID','Integral MPC v3');
end
exportgraphics(fig,fullfile(out,'final_campaign.png'),'Resolution',160);
fig=figure('Name','Current nominal trajectories and inputs','Color','w','Position',[80 80 1150 760]);tiledlayout(2,2);
for m=1:2
 method={'PID','MPC_v3'};L=load(fullfile(out,['LINE_NOM_' method{m} '.mat']),'run');R=load(fullfile(out,['CIRCLE_NOM_' method{m} '.mat']),'run');
 nexttile(1);hold on;mask=L.run.t>=30;plot(L.run.t(mask),L.run.ey(mask),'LineWidth',1.2);
 nexttile(2);hold on;mask=R.run.t>=30;plot(R.run.t(mask),R.run.ey(mask),'LineWidth',1.2);
 nexttile(3);hold on;plot(L.run.t,rad2deg(L.run.z(:,13)),'LineWidth',1.1);
 nexttile(4);hold on;plot(R.run.t,rad2deg(R.run.z(:,13)),'LineWidth',1.1);
end
nexttile(1);grid on;xlabel('Time (s)');ylabel('Line error after 30s (m)');legend('PID','Integral MPC v3');
nexttile(2);grid on;xlabel('Time (s)');ylabel('Circle error after 30s (m)');legend('PID','Integral MPC v3');
nexttile(3);grid on;xlabel('Time (s)');ylabel('Line actual rudder (deg)');
nexttile(4);grid on;xlabel('Time (s)');ylabel('Circle actual rudder (deg)');
exportgraphics(fig,fullfile(out,'final_nominal.png'),'Resolution',160);
disp(P);fprintf('Final v3 campaign: %d/%d engineering targets met, failures %d.\n',summary.engineeringPassed,summary.runCount,summary.solverFailures);
end
