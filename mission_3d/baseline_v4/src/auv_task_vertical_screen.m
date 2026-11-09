function plan=auv_task_vertical_screen(plan,initialDepth)
% Reserve distance for reference-depth acquisition at upper speed .85m/s.
% This screens reference slew time, not six-DOF maneuver feasibility.
task=plan.task;segments=plan.segments;output=struct([]);records={};
previous=[initialDepth,initialDepth];workEntered=false;i=1;
bounded=isfield(task.depth,'workDepthBounds_m');
if bounded
 envelope=task.depth.workDepthBounds_m;
 assert(isnumeric(envelope)&&numel(envelope)==2&&all(isfinite(envelope))&&envelope(1)>=4&&envelope(2)>=envelope(1),'Invalid public work-depth envelope.');
end
while i<=numel(segments)
 s=segments(i);j=i;required=0;target=previous;screen=false;
 if strcmp(s.role,'departure')
  % Initial trim-depth segment; no demand for a different depth.
 elseif strcmp(s.role,'return')
  while j<numel(segments)&&strcmp(segments(j+1).role,'return'),j=j+1;end
  target=[task.depth.return_m,task.depth.return_m];screen=true;
 elseif auv_task_is_point_task(task.type)
  id=s.target;assert(id>=1&&id<=size(plan.targets.positionsNED,1));
  while j<numel(segments)&&~strcmp(segments(j+1).role,'return')&&segments(j+1).target==id,j=j+1;end
  depth=plan.targets.positionsNED(id,3);target=[depth,depth];screen=true;
 elseif ~workEntered
  % First altitude acquisition uses a DECLARED public work-depth envelope.
  while j<numel(segments)&&strcmp(segments(j+1).role,s.role)&&segments(j+1).target==s.target,j=j+1;end
  workEntered=true;
  if bounded,target=reshape(envelope,1,2);screen=true;else,target=[NaN,NaN];end
 end
 parts=segments(i:j);available=sum([parts.length]);extra=segments([]);
 if screen&&all(isfinite([previous,target]))
  delta=max(abs([target(1)-previous(1),target(1)-previous(2),target(2)-previous(1),target(2)-previous(2)]));
  if delta>0,required=(delta/task.depth.maximumRate_m_s+5)*.85;end
  count=max(0,ceil((required-available)/(2*pi*task.minimumTurnRadius_m)));
  if count>0
   pose=[s.start;s.heading0];found=false;
   for direction=[1,-1]
    candidate=d11_segment(pose,'arc',2*pi*task.minimumTurnRadius_m,task.minimumTurnRadius_m,direction,s.role,s.target);
    if d11_chart_clear(candidate,task.chart),found=true;break;end
   end
   assert(found,'No chart-clear full-circle depth-acquisition candidate. Not global infeasibility proof.');
   extra=repmat(candidate,1,count);
  end
  records{end+1}=struct('sourceSegment',i,'targetInterval_m',target,'priorInterval_m',previous,...
   'requiredDistance_m',required,'originalDistance_m',available,'addedLoops',count,...
   'screenedDistance_m',available+sum([extra.length]));
 end
 if isempty(output),output=[extra,parts];else,output=[output,extra,parts];end
 if screen||workEntered,previous=target;end
 i=j+1;
end
plan.segments=output;plan.totalLength_m=sum([output.length]);plan.geometricETA_s=plan.totalLength_m/task.speed_m_s;
plan.withinTimeBudget=plan.geometricETA_s<=task.maximumTime_s;
plan.verticalScreen=struct('records',{records},'altitudeDepthEnvelopeDeclared',bounded,...
 'referenceSlewScreenComplete',auv_task_is_point_task(task.type)||bounded,...
 'scope','Public depth intervals, .85m/s upper speed and5s reference-slew reserve. Added arcs are forward motion, not hovering. Altitude tracking still needs live altimetry and MIL.');
end
