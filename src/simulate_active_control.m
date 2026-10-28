function run = simulate_active_control(C,trim,scenario,h,M)
if nargin<5, M=[]; end
% Deterministic sampled-data simulation; actual 15-state plant is continuous.
P=auv_parameters();
if isfield(scenario,'current'), P.current=scenario.current; end
if isfield(scenario,'kappa'), P.kappa=scenario.kappa; end
if isfield(scenario,'force'), P.force=scenario.force; end
if isfield(scenario,'rudderMax'), P.upper(1)=scenario.rudderMax; P.lower(1)=-scenario.rudderMax; end
if isfield(scenario,'rudderRate'), P.rate(1)=scenario.rudderRate; end
z=[trim.x;trim.a]; z(7)=0; z(8)=scenario.initialError; z(12)=scenario.headingOffset;
if isfield(scenario,'depthInitial'), z(9)=scenario.depthInitial; end
if isfield(scenario,'uInitial'), z(1)=scenario.uInitial; end
memory=control_initial_state(z,trim);
t=(0:C.Ts:scenario.duration)'; n=numel(t);
Z=zeros(n,15); U=zeros(n,3); refs=zeros(n,2); errors=zeros(n,3);
raw=zeros(n,3); integral=zeros(n,3); active=false(n,3); beta=zeros(n,1); ey=zeros(n,1);
amplitudeActive=false(n,3); rateActive=false(n,3); rawHeading=zeros(n,1);
lowSpeed=false(n,1); groundSpeed=zeros(n,1); actuatorDerivative=zeros(n,3);
solverAudit=nan(n,4); mpcLog=nan(n,8); mpcBound=false(n,2); ablationLog=zeros(n,4); nextPrediction=nan(n,2);
count=round(C.Ts/h); assert(abs(count*h-C.Ts)<1e-12);
status='completed'; failure=''; last=n; failureState=[]; failureTime=NaN;
for k=1:n
 Z(k,:)=z';
 [psiRef,memory,g]=ablation_guidance(z(1:12),scenario,memory,C);
 if isfield(scenario,'headingReference')
  % Separate inner-loop training: same bounded smooth reference for all gains.
  psiRef=scenario.headingReference(t(k));
 end
 if isempty(M)
  [command,memory,p]=pid_control_step(z,psiRef,memory,C,P,trim,g.lowSpeed);
 else
  [rudder,memory,mpcInfo]=active_mpc_heading_step(z,psiRef,memory,C,P,trim,M,g.lowSpeed);
  [command,memory,p]=pid_control_step(z,psiRef,memory,C,P,trim,g.lowSpeed,rudder);
  assert(abs(command(1)-rudder)<=1e-8,'AUV:Regulator','QP input altered by external regulator.');
  mpcLog(k,:)=[mpcInfo.exitflag,mpcInfo.solverTime,mpcInfo.modelAndQpTime, ...
   mpcInfo.violation,mpcInfo.failed,mpcInfo.consecutiveFailures,mpcInfo.abort,mpcInfo.iterations];
  mpcBound(k,:)=[mpcInfo.boundActive,mpcInfo.rateActive];
  ablationLog(k,:)=[mpcInfo.integralBefore,mpcInfo.integralAfter,mpcInfo.integralFrozen,mpcInfo.referenceRate];
  nextPrediction(k,:)=mpcInfo.nextPrediction;
  solverAudit(k,:)=[mpcInfo.solverAudit.primaryExitflag,mpcInfo.solverAudit.retryUsed,mpcInfo.solverAudit.retryExitflag,mpcInfo.solverAudit.retryStationarity];
 end
 U(k,:)=command'; refs(k,:)=[psiRef,p.thetaReference]; errors(k,:)=p.error';
 raw(k,:)=p.raw'; integral(k,:)=p.integral'; active(k,:)=p.regulatorActive';
 beta(k)=g.beta; ey(k)=g.ey;
 amplitudeActive(k,:)=p.amplitudeActive'; rateActive(k,:)=p.rateActive';
 rawHeading(k)=g.rawHeading; lowSpeed(k)=g.lowSpeed; groundSpeed(k)=g.groundSpeed;
 actuatorDerivative(k,:)=actuator_rhs(z(13:15),command,P)';
 if ~isempty(M) && mpcInfo.abort
  status='failed'; failure='Three consecutive QP failures'; failureState=z; failureTime=t(k); last=k; break;
 end
 if k==n, break; end
 for j=1:count
  tt=t(k)+(j-1)*h; f=@(time,state) remus_reference_rhs(time,state,command,P);
  k1=f(tt,z); k2=f(tt+h/2,z+h*k1/2); k3=f(tt+h/2,z+h*k2/2); k4=f(tt+h,z+h*k3);
  z=z+h*(k1+2*k2+2*k3+k4)/6;
  if any(~isfinite(z)) || any(abs(z(10:11))>deg2rad(15)) || z(9)<5 || z(9)>15
   status='failed'; failure='Plant outside finite-state/attitude/depth domain'; last=k;
   failureState=z; failureTime=tt+h;
   break;
  end
  assert(all(z(13:15)>=P.lower-1e-8) && all(z(13:15)<=P.upper+1e-8));
 end
 if strcmp(status,'failed'), break; end
end
run=struct('status',status,'failure',failure,'scenario',scenario,'C',C,'h',h, ...
 't',t(1:last),'z',Z(1:last,:),'command',U(1:last,:), ...
 'reference',refs(1:last,:),'controlError',errors(1:last,:), ...
 'rawCommand',raw(1:last,:),'integral',integral(1:last,:), ...
 'constraintActive',active(1:last,:),'beta',beta(1:last),'ey',ey(1:last));
run.amplitudeActive=amplitudeActive(1:last,:); run.rateActive=rateActive(1:last,:);
run.rawHeading=rawHeading(1:last); run.wrappedHeading=angle(exp(1i*Z(1:last,12)));
run.lowSpeed=lowSpeed(1:last); run.groundSpeed=groundSpeed(1:last);
run.actuatorDerivative=actuatorDerivative(1:last,:);
run.failureState=failureState; run.failureTime=failureTime;
run.metrics=pid_metrics(run);
if ~isempty(M)
 run.solverAudit=solverAudit(1:last,:);
 run.ablationLog=ablationLog(1:last,:); run.nextPrediction=nextPrediction(1:last,:);
 run.controller=M.ablation.id; run.M=M; run.mpcLog=mpcLog(1:last,:); run.mpcBoundActive=mpcBound(1:last,:);
 run.mpcColumns={'exitflag','solverTime','modelAndQpTime','violation','failed','consecutiveFailures','abort','iterations'};
 times=run.mpcLog(:,2); sortedTimes=sort(times); idx=max(1,ceil(.95*numel(times)));
 totalTimes=run.mpcLog(:,2)+run.mpcLog(:,3); sortedTotal=sort(totalTimes);
 run.solverMetrics=struct('failureCount',sum(run.mpcLog(:,5)), ...
  'p95_s',sortedTimes(idx),'modelQpTotalP95_s',sortedTotal(idx),'modelQpTotalMax_s',max(totalTimes),'maximum_s',max(times),'overTsFraction',mean(times>C.Ts), ...
  'maxViolation',max(run.mpcLog(:,4)),'maxModelAndQpTime_s',max(run.mpcLog(:,3)), ...
  'boundActiveFraction',mean(run.mpcBoundActive(:,1)), ...
  'rateActiveFraction',mean(run.mpcBoundActive(:,2)));
run.solverMetrics.primaryFailureCount=sum(run.solverAudit(:,1)<=0);
 run.solverMetrics.retryCount=sum(run.solverAudit(:,2)>0);
 run.solverMetrics.retryRecoveredCount=sum(run.solverAudit(:,2)>0 & run.solverAudit(:,3)>0);
else
 run.controller='PID';
end
run.diagnostics=ablation_diagnostics(run);
end
