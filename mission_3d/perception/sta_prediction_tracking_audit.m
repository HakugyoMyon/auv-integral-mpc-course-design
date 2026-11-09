function audit=sta_prediction_tracking_audit(file)
% OFFLINE empirical test of a declared tracking tube, not a safety proof.
% Actual future controls are receding-horizon; the archived prediction is an
% open-loop blocked plan. Their difference is precisely the tested assumption.
z=load(file,'h','gateLog','planner','staticGateLog');h=z.h;
assert(all(diff(h.t)>0)&&size(h.truth,1)==numel(h.t));
tube=z.planner.trackingTube_m;rows=zeros(0,7);accepted=0;skippedTerminal=0;missingPrediction=0;missingNav=0;
for k=1:numel(z.gateLog)
 e=z.gateLog{k};g=e.gate;if ~g.accepted,continue;end;accepted=accepted+1;
 if ~isfield(g,'predictedTrajectory')||~isfield(g,'predictionTimes'),missingPrediction=missingPrediction+1;continue;end
 tau=g.predictionTimes(:);pred=g.predictedTrajectory;
 assert(size(pred,1)==18&&size(pred,2)==numel(tau),'Prediction state/time shape mismatch.');
 assert(abs(tau(1))<1e-9&&all(diff(tau)>0),'Prediction times must start at zero and increase.');
 take=find(tau<=3+1e-8);assert(abs(tau(take(end))-3)<1e-7,'Prediction must cover the complete three-second window.');
 assert(all(abs(diff(tau(take))-.1)<1e-8),'Expected archived 0.1s prediction samples, not internal integration steps.');
 if e.time+3>h.t(end)+1e-8,skippedTerminal=skippedTerminal+1;continue;end
 times=e.time+tau(take);ids=zeros(numel(times),1);
 for j=1:numel(times)
  [gap,ids(j)]=min(abs(h.t-times(j)));assert(gap<1e-7,'Prediction sample has no matching archived 0.1s truth record.');
 end
 assert(numel(unique(ids))==numel(ids),'Duplicate truth-time association.');
 actual=h.truth(ids,1:3);forecast=pred(1:3,take)';
 relative=vecnorm((actual-actual(1,:))-(forecast-forecast(1,:)),2,2);
 direct=vecnorm(actual-forecast,2,2);
 navBound=NaN;
 if isfield(z,'staticGateLog')
  for j=1:numel(z.staticGateLog)
   s=z.staticGateLog{j};if abs(s.time-e.time)<1e-8,navBound=s.navigation.positionErrorBound_m;break;end
  end
 end
 if ~isfinite(navBound),missingNav=missingNav+1;end
 rows=[rows;repmat(k,numel(take),1),repmat(e.time,numel(take),1),tau(take),relative,direct,repmat(navBound,numel(take),1),repmat(navBound+tube,numel(take),1)]; %#ok<AGROW>
end
assert(~isempty(rows),'No accepted gate with a complete archived future window.');
[maximum,peak]=max(rows(:,4));[directMaximum,directPeak]=max(rows(:,5));
budgetSlack=rows(:,7)-rows(:,5);[minimumSlack,slackPeak]=min(budgetSlack);
audit=struct('passed',maximum<=tube+1e-8&&missingNav==0&&missingPrediction==0&&all(budgetSlack>=-1e-8),...
 'accepted_gates',accepted,'evaluated_gates',numel(unique(rows(:,1))),'evaluated_samples',size(rows,1),...
 'skipped_incomplete_future_gates',skippedTerminal,'missing_prediction_gates',missingPrediction,'missing_navigation_budget_gates',missingNav,...
 'tracking_tube_m',tube,'maximum_translation_increment_error_m',maximum,'p95_translation_increment_error_m',percentile95(rows(:,4)),...
 'increment_peak_gate_time_s',rows(peak,2),'increment_peak_prediction_tau_s',rows(peak,3),'increment_tube_pass',maximum<=tube+1e-8,...
 'maximum_direct_position_error_m',directMaximum,'p95_direct_position_error_m',percentile95(rows(:,5)),...
 'direct_peak_gate_time_s',rows(directPeak,2),'direct_peak_prediction_tau_s',rows(directPeak,3),...
 'minimum_initial_navigation_plus_tube_slack_m',minimumSlack,'budget_peak_gate_time_s',rows(slackPeak,2),'budget_peak_prediction_tau_s',rows(slackPeak,3),...
 'direct_navigation_plus_tube_pass',missingNav==0&&all(budgetSlack>=-1e-8),...
 'scope','Empirical archived complete-window 0:0.1:3s samples. Translation increment removes the initial position offset but retains attitude, navigation evolution and receding-horizon command differences. Direct error uses each capture-time static gate navigation position bound plus tracking tube, not final planner budget. This does not establish statistical confidence, unseen-condition robustness, body attitude tube, or continuous-time safety.');
folder=fileparts(file);fid=fopen(fullfile(folder,'prediction_tracking_audit.json'),'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true));
names={'gateIndex','gateTime_s','predictionTau_s','incrementError_m','directError_m','initialNavigationBound_m','navigationPlusTubeBudget_m'};
writetable(array2table(rows,'VariableNames',names),fullfile(folder,'prediction_tracking_samples.csv'));
disp(audit);
end

function value=percentile95(x)
% Explicit linearly interpolated empirical percentile, no Statistics Toolbox.
x=sort(x(:));index=1+.95*(numel(x)-1);lo=floor(index);hi=ceil(index);value=x(lo)+(index-lo)*(x(hi)-x(lo));
end
