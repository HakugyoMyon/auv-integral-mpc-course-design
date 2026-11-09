function audit=sta_body_prediction_audit(file)
% Empirical full-body-center prediction tube test, discrete archived samples.
z=load(file,'h','gateLog','staticGateLog','planner');h=z.h;tube=z.planner.trackingTube_m;
assert(all(diff(h.t)>0)&&size(h.truth,1)==numel(h.t));
% Seven centers consistently cover the longitudinal body box [-1.30,.65].
xs=linspace(-1.30,.65,7);lever=max(abs(xs));
accepted=0;evaluated=0;skipped=0;missing=0;sampleCount=0;
maxIncrement=-Inf;maxDirect=-Inf;minSlack=Inf;incrementViolations=0;directViolations=0;
incrementPeak=struct();directPeak=struct();slackPeak=struct();
for k=1:numel(z.gateLog)
 e=z.gateLog{k};g=e.gate;if ~g.accepted,continue;end;accepted=accepted+1;
 if e.time+3>h.t(end)+1e-8,skipped=skipped+1;continue;end
 if ~isfield(g,'predictedTrajectory')||~isfield(g,'predictionTimes'),missing=missing+1;continue;end
 tau=g.predictionTimes(:);prediction=g.predictedTrajectory;
 if size(prediction,1)~=18||size(prediction,2)~=numel(tau)||isempty(tau)||abs(tau(1))>1e-8||any(diff(tau)<=0)
  missing=missing+1;continue;
 end
 take=find(tau<=3+1e-8);
 if isempty(take)||abs(tau(take(end))-3)>1e-8||any(abs(diff(tau(take))-.1)>1e-8),missing=missing+1;continue;end
 ids=zeros(numel(take),1);okay=true;
 for j=1:numel(take),[gap,ids(j)]=min(abs(h.t-(e.time+tau(take(j)))));okay=okay&&gap<1e-7;end
 if ~okay||numel(unique(ids))~=numel(ids),missing=missing+1;continue;end
 nav=[];for j=1:numel(z.staticGateLog)
  s=z.staticGateLog{j};if abs(s.time-e.time)<1e-8,nav=s.navigation;break;end
 end
 if isempty(nav)||~all(isfield(nav,{'positionErrorBound_m','attitudeErrorBound_rad'}))||...
   any(~isfinite([nav.positionErrorBound_m,nav.attitudeErrorBound_rad]))||nav.positionErrorBound_m<0||nav.attitudeErrorBound_rad<0
  missing=missing+1;continue;
 end
 budget=nav.positionErrorBound_m+2*lever*sin(nav.attitudeErrorBound_rad/2)+tube;
 actual=zeros(3,numel(xs),numel(take));forecast=actual;
 for j=1:numel(take)
  xt=h.truth(ids(j),:)';xp=prediction(:,take(j));
  actual(:,:,j)=xt(1:3)+auv6_rotation(xt(4:7))*[xs;zeros(2,numel(xs))];
  forecast(:,:,j)=xp(1:3)+auv6_rotation(xp(4:7))*[xs;zeros(2,numel(xs))];
 end
 if any(~isfinite(actual),'all')||any(~isfinite(forecast),'all'),missing=missing+1;continue;end
 evaluated=evaluated+1;
 for j=1:numel(take)
  increment=vecnorm((actual(:,:,j)-actual(:,:,1))-(forecast(:,:,j)-forecast(:,:,1)),2,1);
  direct=vecnorm(actual(:,:,j)-forecast(:,:,j),2,1);slack=budget-direct;
  sampleCount=sampleCount+numel(xs);incrementViolations=incrementViolations+sum(increment>tube+1e-8);directViolations=directViolations+sum(slack< -1e-8);
  [v,ix]=max(increment);if v>maxIncrement,maxIncrement=v;incrementPeak=peak(k,e.time,tau(take(j)),xs(ix),v,budget);end
  [v,ix]=max(direct);if v>maxDirect,maxDirect=v;directPeak=peak(k,e.time,tau(take(j)),xs(ix),v,budget);end
  [v,ix]=min(slack);if v<minSlack,minSlack=v;slackPeak=peak(k,e.time,tau(take(j)),xs(ix),direct(ix),budget);end
 end
end
audit=struct('passed',evaluated>0&&missing==0&&incrementViolations==0&&directViolations==0,...
 'accepted_gates',accepted,'evaluated_gates',evaluated,'skipped_terminal_incomplete_future_gates',skipped,...
 'missing_or_malformed_gates',missing,'body_center_samples',sampleCount,'bodyX_m',xs,'trackingTube_m',tube,...
 'maximum_body_translation_increment_error_m',maxIncrement,'increment_peak',incrementPeak,...
 'maximum_direct_body_center_error_m',maxDirect,'direct_peak',directPeak,...
 'minimum_initial_navigation_attitude_plus_tube_slack_m',minSlack,'minimum_slack_peak',slackPeak,...
 'increment_tube_violations',incrementViolations,'direct_budget_violations',directViolations,...
 'scope','Discrete empirical complete 0:0.1:3s archived accepted-gate test of seven longitudinal body centers. Increment error includes future attitude evolution after removing initial center offset. Direct budget uses each gate navigation position bound plus 2*max(abs(bodyX))*sin(attitudeBound/2) plus trackingTube. Not a statistical calibration, continuous-time bound, exhaustive-condition guarantee or replacement for full-body collision audit.');
fid=fopen(fullfile(fileparts(file),'body_prediction_audit.json'),'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true));disp(audit);
end

function value=peak(index,time,tau,bodyX,error,budget)
value=struct('gateIndex',index,'gateTime_s',time,'predictionTau_s',tau,'bodyX_m',bodyX,'error_m',error,'directBudget_m',budget);
end
