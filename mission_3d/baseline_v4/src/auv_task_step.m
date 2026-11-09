function [mission,status]=auv_task_step(mission,plan,xhat,quality,depthSigma,time,altitude,payloadLedger)
% Actual payload task, separate from visit-only bookkeeping.
if isempty(mission)
 mission=struct('segment',1,'progress',0,'filteredSideslip',0,'finished',false,'complete',false,...
  'returnVerified',false,'bottomDepth',NaN,'lastBottomTime',-Inf);
end
for transition=1:numel(plan.segments)
 segment=plan.segments(mission.segment);
 [progress,error,projection,heading,raw]=d11_path_project(xhat(1:2),segment,mission.progress);mission.progress=progress;
 reached=raw>=segment.length-.8&&norm(xhat(1:2)-segment.finish)<=3;
 if reached&&mission.segment<numel(plan.segments),mission.segment=mission.segment+1;mission.progress=0;else,break;end
end
if altitude.valid&&altitude.time>mission.lastBottomTime
 if isnan(mission.bottomDepth),mission.bottomDepth=altitude.bottomDepth;
 else
  dt=altitude.time-mission.lastBottomTime;mission.bottomDepth=mission.bottomDepth+dt/(2+dt)*(altitude.bottomDepth-mission.bottomDepth);
 end
 mission.lastBottomTime=altitude.time;
end
segment=plan.segments(mission.segment);depthTarget=plan.task.depth.transit_m;
workMode=~any(strcmp(segment.role,{'departure','return'}));heightMissing=false;
if strcmp(segment.role,'return'),depthTarget=plan.task.depth.return_m;
elseif workMode&&auv_task_is_point_task(plan.task.type)
 assert(segment.target>=1&&segment.target<=size(plan.targets.positionsNED,1),'Visit segment missing target identity.');
 depthTarget=plan.targets.positionsNED(segment.target,3);
elseif workMode
 heightMissing=~isfinite(mission.bottomDepth)||time-mission.lastBottomTime>.7;
 if ~heightMissing,depthTarget=mission.bottomDepth-plan.task.depth.workAltitude_m;end
end
routeExhausted=mission.segment==numel(plan.segments)&&reached;
if ~isfield(mission,'terminalApproachActive'),mission.terminalApproachActive=false;end
if mission.segment<numel(plan.segments),mission.terminalApproachActive=false;end
mission.terminalApproachActive=mission.terminalApproachActive||routeExhausted;
mission.terminalVerified=false;mission.returnVerified=false;mission.finished=false;mission.complete=false;
rejected=false;
if mission.terminalApproachActive
 [goal,kind]=auv_task_terminal(plan);
 mission.terminalVerified=quality.valid&&norm(xhat(1:2)-goal(1:2))+quality.radius95<=5&&abs(xhat(3)-goal(3))+3*depthSigma+.05<=.5;
 mission.returnVerified=mission.terminalVerified&&strcmp(kind,'home');
 endpointOperation=any(strcmp(plan.task.type,{'transit','return'}));
 if isfield(plan,'operatorReturn')&&plan.operatorReturn,endpointOperation=false;end
 mission.finished=mission.terminalVerified&&(~endpointOperation||payloadLedger.complete);
 mission.complete=mission.finished&&~isempty(payloadLedger)&&payloadLedger.complete;
 rejected=~mission.finished&&raw>segment.length+5;
end
outsideDepthEnvelope=false;
if workMode&&~auv_task_is_point_task(plan.task.type)&&isfield(plan.task.depth,'workDepthBounds_m')&&~heightMissing
 bounds=plan.task.depth.workDepthBounds_m;outsideDepthEnvelope=depthTarget<bounds(1)||depthTarget>bounds(2);
end
status=struct('crossTrack',error,'projection',projection,'tangentHeading',heading,'depthTarget',depthTarget,...
 'routeExhausted',routeExhausted,'returnConfirmationRejected',rejected,...
 'heightMissing',heightMissing,'workMode',workMode,'outsideDepthEnvelope',outsideDepthEnvelope);
end
