function summary = summarize_sensor_study_v3()
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'results','sensor_study_v3');
T=[readtable(fullfile(out,'metrics_cases_1.csv'),'TextType','string'); ...
 readtable(fullfile(out,'metrics_cases_2.csv'),'TextType','string');readtable(fullfile(out,'metrics_cases_11.csv'),'TextType','string')];
assert(height(T)==42);writetable(T,fullfile(out,'all_run_metrics.csv'));
cases={'LINE_NOM','CIRCLE_NOM','LINE_TIGHT'};conditions={'delay','noise','noise_delay'};
groups=struct([]);pairs=struct([]);checks=struct([]);
for i=1:3
 for j=1:3
  mask=T.caseId==cases{i} & T.condition==conditions{j};S=T(mask,:);
  seeds=unique(S.seed);differences=zeros(numel(seeds),2);
  for k=1:numel(seeds)
   p=S(S.seed==seeds(k) & S.method=='PID',:);m=S(S.seed==seeds(k) & S.method=='MPC_v3',:);
   assert(height(p)==1 && height(m)==1);
   tag=[cases{i} '_' conditions{j} '_' num2str(seeds(k))];
   a=load(fullfile(out,[tag '_PID.mat']),'run');b=load(fullfile(out,[tag '_MPC_v3.mat']),'run');
   assert(isequal(a.run.acquisitionNoise,b.run.acquisitionNoise));
   for run={a.run,b.run}
    rr=run{1};z=rr.z;expected=z(:,8);
    if strcmp(cases{i},'CIRCLE_NOM'),expected=50-sqrt(z(:,7).^2+(z(:,8)-50).^2);end
    assert(max(abs(rr.ey-expected))<1e-12);
    idx=rr.measurementIndex;
    assert(max(abs(rr.measuredState(:,1:12)-rr.z(idx,1:12)-rr.acquisitionNoise(idx,:)),[],'all')<1e-12);
    assert(max(abs(rr.measuredState(:,13:15)-z(:,13:15)),[],'all')==0);
   end
   differences(k,:)=[m.rmsePost30-p.rmsePost30,m.rudderTVrad-p.rudderTVrad];
   pair=struct('caseId',cases{i},'condition',conditions{j},'seed',seeds(k), ...
    'pidRmse',p.rmsePost30,'mpcRmse',m.rmsePost30,'deltaRmse',differences(k,1), ...
    'pidTV',p.rudderTVrad,'mpcTV',m.rudderTVrad,'deltaTV',differences(k,2));
   if isempty(pairs),pairs=pair;else,pairs(end+1)=pair;end %#ok<AGROW>
  end
  for method={'PID','MPC_v3'}
   R=S(S.method==method{1},:);item=struct('caseId',cases{i},'condition',conditions{j},'method',method{1}, ...
    'n',height(R),'rmseMean',mean(R.rmsePost30),'rmseStd',std(R.rmsePost30), ...
    'TVmean',mean(R.rudderTVrad),'TVstd',std(R.rudderTVrad),'steadyRmseMean',mean(R.steadyRmse), ...
    'completed',sum(R.status=='completed'),'finalFailures',sum(R.finalQpFailures), ...
    'pairedRmseDifferenceMean',mean(differences(:,1)),'pairedRmseDifferenceStd',std(differences(:,1)));
   if isempty(groups),groups=item;else,groups(end+1)=item;end %#ok<AGROW>
  end
 end
end
limits=20*ones(height(T),1);rates=limits;mask=T.caseId=='LINE_TIGHT';limits(mask)=10;rates(mask)=5;
assert(all(T.rudderPeakDeg<=limits+1e-6) && all(T.commandRateDegS<=rates+1e-6));
summary=struct('status','completed','newRuns',42,'pairedComparisons',21,'allCompleted',all(T.status=='completed'), ...
 'finalQpFailures',sum(T.finalQpFailures),'sameQpRetries',sum(T.sameQpRetries),'maxInputViolation',max(T.inputViolation), ...
 'pairedNoiseVerified',true,'truthMetricsVerified',true,'measurementDelayVerified',true, ...
 'mpcRmseLowerPairs',sum([pairs.deltaRmse]<0),'mpcTVLowerPairs',sum([pairs.deltaTV]<0), ...
 'groups',groups,'pairs',pairs);
writetable(struct2table(groups),fullfile(out,'group_metrics.csv'));writetable(struct2table(pairs),fullfile(out,'paired_metrics.csv'));
save(fullfile(out,'sensor_summary.mat'),'summary','T');
fid=fopen(fullfile(out,'sensor_summary.json'),'w');c=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));
base=jsondecode(fileread(fullfile(root,'docs','results','final_comparison_summary.json')));
fig=figure('Visible','off','Color','w','Position',[30 30 1400 800]);tiledlayout(2,3);
for i=1:3
 rmse=zeros(4,2);tv=rmse;rmseStd=rmse;tvStd=rmse;
 for m=1:2
  methods={'PID','MPC_v3'};rr=base.metrics(strcmp({base.metrics.caseId},cases{i}) & strcmp({base.metrics.method},methods{m}));
  rmse(1,m)=rr.rmsePost30;tv(1,m)=rr.rudderTVrad;
  for j=1:3
   rr=groups(strcmp({groups.caseId},cases{i}) & strcmp({groups.condition},conditions{j}) & strcmp({groups.method},methods{m}));
   rmse(j+1,m)=rr.rmseMean;tv(j+1,m)=rr.TVmean;rmseStd(j+1,m)=rr.rmseStd;tvStd(j+1,m)=rr.TVstd;
  end
 end
 nexttile(i);B=bar(rmse);hold on;
 for m=1:2,errorbar(B(m).XEndPoints,rmse(:,m),rmseStd(:,m),'k.','LineWidth',1);end
 grid on;xticklabels({'Ideal','Delay','Noise','Noise+delay'});ylabel('Truth post-30s RMSE (m)');title(strrep(cases{i},'_',' '));legend('PID','MPC v3');
 nexttile(i+3);B=bar(tv);hold on;
 for m=1:2,errorbar(B(m).XEndPoints,tv(:,m),tvStd(:,m),'k.','LineWidth',1);end
 grid on;xticklabels({'Ideal','Delay','Noise','Noise+delay'});ylabel('Actual rudder TV (rad)');
end
exportgraphics(fig,fullfile(out,'sensor_study_comparison.png'),'Resolution',160);close(fig);
fprintf('SENSOR SUMMARY: %d complete, QP failures%d, MPC lower RMSE %d/21, lower TV %d/21.\n', ...
 height(T),summary.finalQpFailures,summary.mpcRmseLowerPairs,summary.mpcTVLowerPairs);
end
