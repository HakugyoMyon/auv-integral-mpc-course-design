function [rudder,memory,info] = preview_candidate_heading_step(z,reference,memory,C,P,trim,M,low,geometricRate,forceFailure)
if nargin<10, forceFailure=false; end
if ~isfield(memory,'mpcConsecutiveFailures'), memory.mpcConsecutiveFailures=0; end
if ~isfield(memory,'ablationIntegral'), memory.ablationIntegral=0; end
if isfield(memory,'lastAblationReference')
 rate=angle(exp(1i*(reference-memory.lastAblationReference)))/C.Ts;
 rate=min(max(rate,-C.psiRate),C.psiRate);
else
 rate=0;
end
legacyRate=rate;
switch M.previewPolicy
 case 'hold',rate=0;
 case 'geometry',rate=min(max(geometricRate,-C.psiRate),C.psiRate);
 otherwise,error('AUV:PreviewPolicy','Unknown preview policy.');
end
if low,rate=0;end
memory.lastAblationReference=reference;
previous=memory.previousCommand(1); eta=memory.ablationIntegral; started=tic;
model=ablation_prediction_model(trim,C.Ts,z,previous,M.ablation.dimension,false);
qp=build_ablation_qp(model,M,reference,previous,P,rate,eta);
modelTime=toc(started); solverStarted=tic;
audit=struct('primaryExitflag',-99,'retryUsed',false,'retryExitflag',NaN,'retryStationarity',NaN);
if forceFailure
 decision=[]; exitflag=-99; output=struct('iterations',0);
else
 [decision,exitflag,output,audit]=solve_current_heading_qp(qp,M);
end
solverTime=toc(solverStarted); violation=Inf;
if numel(decision)==M.Nc && all(isfinite(decision))
 plan=previous+cumsum(decision*M.incrementScale);
 violation=max([0;plan-P.upper(1);P.lower(1)-plan;abs(diff([previous;plan]))-P.rate(1)*C.Ts]);
end
failed=exitflag<=0 || ~isfinite(violation) || violation>M.constraintTolerance;
if failed
 rudder=previous; memory.mpcConsecutiveFailures=memory.mpcConsecutiveFailures+1;
else
 rudder=plan(1); memory.mpcConsecutiveFailures=0;
end
boundActive=abs(rudder-P.lower(1))<1e-7 || abs(rudder-P.upper(1))<1e-7;
rateActive=abs(abs(rudder-previous)-P.rate(1)*C.Ts)<1e-7;
err=angle(exp(1i*(reference-z(12)))); frozen=false;
if M.ablation.integral
 atUpper=rudder>=P.upper(1)-1e-7 && err>0;
 atLower=rudder<=P.lower(1)+1e-7 && err<0;
 pushingRate=rateActive && (rudder-previous)*err>0;
 frozen=low || failed || atUpper || atLower || pushingRate;
 if ~frozen
  memory.ablationIntegral=min(max(eta+C.Ts*err,-M.ablation.integralLimit),M.ablation.integralLimit);
 end
end
pred=model.origin+model.gd+model.Bd*(rudder-model.inputOrigin);
info=struct('exitflag',exitflag,'solverTime',solverTime,'modelAndQpTime',modelTime, ...
 'violation',violation,'failed',failed,'consecutiveFailures',memory.mpcConsecutiveFailures, ...
 'abort',memory.mpcConsecutiveFailures>=3,'iterations',output.iterations, ...
 'boundActive',boundActive,'rateActive',rateActive,'decision',decision, ...
 'legacyRate',legacyRate,'geometricRate',geometricRate,'referenceRate',rate,'integralBefore',eta,'integralAfter',memory.ablationIntegral, ...
 'integralFrozen',frozen,'nextPrediction',[pred(model.psiIndex),pred(model.rateIndex)],'solverAudit',audit);
end
