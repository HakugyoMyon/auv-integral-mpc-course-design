function [mission,status]=sta_step(mission,route,xhat,quality,sigma,time,altitude,ledger)
% Local detours never pretend to be work targets. Original ledger is external.
if ~isfield(route,'staticAvoidance')||~route.staticAvoidance
 [mission,status]=auv_task_step(mission,route,xhat,quality,sigma,time,altitude,ledger);return;
end
if isempty(mission)
 mission=struct('segment',1,'progress',0,'filteredSideslip',0,'finished',false,'complete',false,...
  'returnVerified',false,'bottomDepth',NaN,'lastBottomTime',-Inf);
end
for transition=1:numel(route.segments)
 segment=route.segments(mission.segment);
 [progress,error,projection,heading,raw]=d11_path_project(xhat(1:2),segment,mission.progress);mission.progress=progress;
 reached=raw>=segment.length-.8&&norm(xhat(1:2)-segment.finish)<=3;
 if reached&&mission.segment<numel(route.segments),mission.segment=mission.segment+1;mission.progress=0;else,break;end
end
mission.finished=mission.segment==numel(route.segments)&&reached;
mission.complete=false;mission.returnVerified=false;mission.terminalVerified=false;
mission.terminalApproachActive=false;
status=struct('crossTrack',error,'projection',projection,'tangentHeading',heading,...
 'depthTarget',route.depthTargets(mission.segment),'workMode',false,'heightMissing',false,...
 'outsideDepthEnvelope',false,'routeExhausted',mission.finished,'returnConfirmationRejected',false);
end
