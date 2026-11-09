function [mission,status]=sta_step_height(mission,route,xhat,quality,sigma,time,altitude,ledger)
% Local detours never pretend to be work targets. Original ledger is external.
if ~isfield(route,'staticAvoidance')||~route.staticAvoidance
 [mission,status]=auv_task_step(mission,route,xhat,quality,sigma,time,altitude,ledger);
 status.executionVariant='altitude_inheritance_v1';return;
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
% Preserve live bottom evidence even in tactical vertical/helical modes.
if altitude.valid&&altitude.time>mission.lastBottomTime
 if isnan(mission.bottomDepth),mission.bottomDepth=altitude.bottomDepth;
 else
  dt=altitude.time-mission.lastBottomTime;
  mission.bottomDepth=mission.bottomDepth+dt/(2+dt)*(altitude.bottomDepth-mission.bottomDepth);
 end
 mission.lastBottomTime=altitude.time;
end
inherit=isfield(route,'inheritAltitudeMode')&&route.inheritAltitudeMode;
depthTarget=route.depthTargets(mission.segment);heightMissing=false;outsideDepthEnvelope=false;
if inherit
 assert(isfield(route,'altitudeDepthBounds_m')&&numel(route.altitudeDepthBounds_m)==2);
 bounds=route.altitudeDepthBounds_m;
 assert(all(isfinite(bounds))&&bounds(1)<=bounds(2),'Empty altitude/planner depth intersection.');
 heightMissing=~isfinite(mission.bottomDepth)||time-mission.lastBottomTime>.7;
 if ~heightMissing
  depthTarget=mission.bottomDepth-route.task.depth.workAltitude_m;
  outsideDepthEnvelope=depthTarget<bounds(1)||depthTarget>bounds(2);
 end
end
mission.finished=mission.segment==numel(route.segments)&&reached;
mission.complete=false;mission.returnVerified=false;mission.terminalVerified=false;
mission.terminalApproachActive=false;
status=struct('crossTrack',error,'projection',projection,'tangentHeading',heading,...
 'depthTarget',depthTarget,'workMode',inherit,'heightMissing',heightMissing,...
 'outsideDepthEnvelope',outsideDepthEnvelope,'routeExhausted',mission.finished,'returnConfirmationRejected',false,...
 'executionVariant','altitude_inheritance_v1');
end
