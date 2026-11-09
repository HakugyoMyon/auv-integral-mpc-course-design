function [check,plan]=auv_task_validate(mission)
% Public configuration -> existing geometric planner. No MIL is launched.
check=struct('valid',false,'errors',{{}},'geometricETA_s',NaN,...
 'scope','Input and nominal geometry/time checks only; not control, observation, energy or whole-mission qualification.');plan=[];
try
 assert(isstruct(mission)&&isscalar(mission)&&strcmp(mission.schema,'auv.mission.v1'),'Invalid mission schema.');
 assert(all(ismember(fieldnames(mission),{'schema','task','execution'})),'Unknown mission fields.');
 e=mission.execution;
 faultFields={'payloadUnavailableWindows_s','dvlUnavailableWindows_s','altimeterUnavailableWindows_s'};
 assert(all(ismember(fieldnames(e),[{'environment','seed','avoidanceEnabled'},faultFields])),'Unknown execution fields.');
 for fi=1:numel(faultFields)
 if isfield(e,faultFields{fi})&&~isempty(e.(faultFields{fi}))
  w=e.(faultFields{fi});
  assert(isnumeric(w)&&size(w,2)==2&&size(w,1)<=20&&all(isfinite(w),'all')&&all(w(:,1)>=0)&&all(w(:,2)>w(:,1)),'Fault windows must be at most20 finite [start,end] rows with0<=start<end.');
 end
 end
 assert(strcmp(e.environment,'obstacle_free')&&isequal(e.avoidanceEnabled,false),'Basic-task stage requires obstacle-free execution; avoidance development is frozen.');
 assert(isnumeric(e.seed)&&isscalar(e.seed)&&isfinite(e.seed)&&e.seed>=0&&e.seed<=2^31-1&&e.seed==floor(e.seed),'Invalid seed.');
 mission=auv_task_normalize(mission);t=mission.task;
 assert(isscalar(t.maximumTime_s)&&isfinite(t.maximumTime_s)&&t.maximumTime_s>0&&t.maximumTime_s<=7200,'Supported task duration is positive and at most7200s.');
 assert(isempty(t.chart.knownCircles),'Obstacle-free stage does not accept chart obstacle inputs.');
 b=t.chart.operatingBounds;assert(all(isfinite(b))&&b(1)<b(2)&&b(3)<b(4),'Invalid operating bounds.');
 d=t.depth;assert(all(isfinite([d.transit_m,d.return_m,d.workAltitude_m,d.maximumRate_m_s]))&&d.transit_m>=4&&d.return_m>=4,'Invalid depth settings.');
 if auv_task_is_point_task(t.type),assert(strcmp(d.workMode,'depth'),'Visits require depth mode.');else,assert(strcmp(d.workMode,'altitude'),'Survey/inspection require altitude mode.');end
 assert(t.speed_m_s==.8,'Only the0.8m/s MPC operating branch is currently supported.');
 assert(all(isfinite(t.startPose))&&norm(t.startPose)<1e-12,'Current navigation alignment fixture requires the local origin and initial heading0.');
 allowed={'schema','id','type','startPose','homePose','speed_m_s','minimumTurnRadius_m','departureLength_m','maximumTime_s','chart','depth','payload','survey','inspection','visit','transit','homeReturnHeadingRequired'};
 assert(all(ismember(fieldnames(t),allowed)),'Unknown/private task fields.');
 assert(all(isfinite([t.minimumTurnRadius_m,t.departureLength_m])),'Nonfinite path settings.');
 if strcmp(t.type,'visit'),assert(size(t.visit.pointsNED,1)<=50,'Initial release supports at most50 visit points.');end
 if strcmp(t.type,'survey')
  g=t.payload.gridResolution_m;assert(isscalar(g)&&isfinite(g)&&g>0,'Invalid grid size.');
  assert(all(isfinite([t.survey.length_m,t.survey.width_m]))&&t.survey.length_m>0&&t.survey.width_m>0,'Invalid survey dimensions.');
  assert(ceil(t.survey.length_m/g)*ceil(t.survey.width_m/g)<=2000,'Initial release supports at most2000 coverage cells.');
 end
 if strcmp(t.type,'inspection')
  points=t.inspection.polylineNE;spacing=t.inspection.pointSpacing_m;assert(size(points,1)<=50&&all(isfinite(points),'all')&&isscalar(spacing)&&isfinite(spacing)&&spacing>0,'Invalid inspection line or spacing.');
  assert(ceil(sum(vecnorm(diff(points),2,2))/spacing)+1<=2000,'Initial release supports at most2000 inspection targets.');
 end
 auv_task_check_capability(t);
 plan=auv_task_plan(t);
 assert(plan.allTargetsGeometricallyPlanned,'Some targets have no geometric route.');
 assert(plan.withinTimeBudget,'Nominal route exceeds task time budget.');
 check.geometricETA_s=plan.geometricETA_s;check.valid=true;
catch ex
 check.errors={ex.message};plan=[];
end
end
