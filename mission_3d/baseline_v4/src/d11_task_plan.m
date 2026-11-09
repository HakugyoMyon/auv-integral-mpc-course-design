function plan=d11_task_plan(task)
% Structured real-task geometry and accounting targets; never environment truth.
assert(strcmp(task.schema,'auv.actual_task.v1'));
allowed={'schema','id','type','startPose','homePose','speed_m_s','minimumTurnRadius_m','departureLength_m','maximumTime_s','chart','depth','payload','survey','inspection','visit'};
assert(all(ismember(fieldnames(task),allowed)),'Unknown/private task field.');
assert(any(strcmp(task.type,{'survey','inspection','visit'})));
assert(task.speed_m_s==.8 && task.minimumTurnRadius_m>=25 && task.departureLength_m>=8);
assert(task.depth.maximumRate_m_s>0&&task.depth.maximumRate_m_s<=.06);
assert(task.maximumTime_s>0&&task.chart.clearance_m>=7);
assert(numel(task.chart.operatingBounds)==4&&(isempty(task.chart.knownCircles)||size(task.chart.knownCircles,2)==3));
if isempty(task.chart.knownCircles),task.chart.knownCircles=zeros(0,3);end
assert(all(ismember(fieldnames(task.chart),{'operatingBounds','knownCircles','clearance_m'})));
assert(~isfield(task.chart,'truth')&&~isfield(task.chart,'dynamicObstacles'));
assert(all(isfinite(task.startPose))&&all(isfinite(task.homePose)));
plan=struct('schema','auv.actual_task_plan.v1','task',task,'segments',struct([]),'targets',struct([]),...
 'connections',struct([]),'geometricCandidateOnly',true,'controllerValidated',false,...
 'verticalFeasibilityValidated',false,'unplannedTargets',[],'totalLength_m',0,'geometricETA_s',Inf,'withinTimeBudget',false);
pose=task.startPose(:);lastDepth=task.depth.transit_m;
departure=d11_segment(pose,'line',task.departureLength_m,Inf,0,'departure',0);
assert(d11_chart_clear(departure,task.chart),'Departure does not fit the known operating chart.');append(departure);
switch task.type
 case 'survey'
  spec=task.survey;assert(strcmp(task.depth.workMode,'altitude')&&task.depth.workAltitude_m>=5);
  assert(spec.length_m>0&&spec.width_m>0&&spec.maximumLineSpacing_m>0&&spec.requiredCoverage>0&&spec.requiredCoverage<=1);
  h=task.depth.workAltitude_m;payload=task.payload;
  designMargin=payload.nominalNavigationMargin_m+sqrt(2)*payload.gridResolution_m/2+.3;
  effectiveWidth=2*(h*tan(payload.crossHalfAngle_rad)-designMargin);
  assert(effectiveWidth>0&&h*sqrt(1+tan(payload.crossHalfAngle_rad)^2+tan(payload.alongHalfAngle_rad)^2)<=payload.maximumRange_m);
  maximumSpacing=min(spec.maximumLineSpacing_m,effectiveWidth);count=max(1,ceil(spec.width_m/maximumSpacing));
  offsets=((1:count)-.5)*spec.width_m/count-spec.width_m/2;
  forward=[cos(spec.bearing_rad);sin(spec.bearing_rad)];right=[-sin(spec.bearing_rad);cos(spec.bearing_rad)];
  for lane=1:count
   sign=1;if mod(lane,2)==0,sign=-1;end
   start=spec.centerNE(:)-sign*spec.length_m/2*forward+offsets(lane)*right;
   heading=spec.bearing_rad+double(sign<0)*pi;
   connectTo([start;heading],'survey_approach',lane);
   append(d11_segment(pose,'line',spec.length_m,Inf,0,'survey',lane));
  end
  [along,across]=ndgrid((-spec.length_m/2+payload.gridResolution_m/2):payload.gridResolution_m:spec.length_m/2,...
   (-spec.width_m/2+payload.gridResolution_m/2):payload.gridResolution_m:spec.width_m/2);
  centers=spec.centerNE(:)'+along(:)*forward'+across(:)*right';
  plan.targets=struct('kind','coverage_cells','centersNE',centers,'cellSize_m',payload.gridResolution_m,...
   'requiredFraction',spec.requiredCoverage,'minimumObservations',payload.minimumObservations);
 case 'inspection'
  spec=task.inspection;points=spec.polylineNE;assert(size(points,2)==2&&size(points,1)>=2&&all(isfinite(points),'all'));
  assert(strcmp(task.depth.workMode,'altitude')&&task.depth.workAltitude_m>=5&&spec.pointSpacing_m>0);
  segmentLength=vecnorm(diff(points),2,2);assert(all(segmentLength>0));progress=[0;cumsum(segmentLength)];
  distances=unique([0:spec.pointSpacing_m:progress(end),progress(end)]);
  targets=interp1(progress,points,distances);plan.targets=struct('kind','inspection_observation_points','positionsNE',targets,...
   'minimumObservations',spec.minimumObservations,'interpretation','Geometrically valid observations of known facility points; no defect-recognition algorithm.');
  heading=atan2(points(2,2)-points(1,2),points(2,1)-points(1,1));connectTo([points(1,:)';heading],'inspection_approach',1);
  for k=2:size(points,1)
   heading=atan2(points(k,2)-points(k-1,2),points(k,1)-points(k-1,1));
   connectTo([points(k,:)';heading],'inspection',k);
  end
 case 'visit'
  spec=task.visit;assert(~spec.hoverRequired,'Forward-only AUV cannot promise stationary hovering.');
  points=spec.pointsNED;assert(size(points,2)==3&&all(isfinite(points),'all')&&all(points(:,3)>=4));
  priorities=spec.priority(:);assert(numel(priorities)==size(points,1));remaining=1:size(points,1);order=[];
  while ~isempty(remaining)
   eligible=remaining(priorities(remaining)==max(priorities(remaining)));bestCost=Inf;best=[];bestParts=[];bestAudit=[];
   for index=eligible
    delta=points(index,1:2)'-pose(1:2);heading=atan2(delta(2),delta(1));if norm(delta)<1e-8,heading=pose(3);end
    [parts,audit]=d11_connect(pose,[points(index,1:2)';heading],task.minimumTurnRadius_m,task.chart,'visit_approach',index);
    if audit.accepted&&audit.length_m<bestCost,best=index;bestCost=audit.length_m;bestParts=parts;bestAudit=audit;end
   end
   if isempty(best),plan.unplannedTargets=remaining;break;end
   requiredTime=abs(points(best,3)-lastDepth)/task.depth.maximumRate_m_s+5;
   if bestCost/task.speed_m_s<requiredTime
    heading=atan2(points(best,2)-pose(2),points(best,1)-pose(1));
    [longer,longAudit]=d11_connect(pose,[points(best,1:2)';heading],task.minimumTurnRadius_m,task.chart,'visit_approach',best,requiredTime*task.speed_m_s);
    if longAudit.accepted,bestParts=longer;bestAudit=longAudit;
    else
     loops=ceil((requiredTime-bestCost/task.speed_m_s)/(2*pi*task.minimumTurnRadius_m/task.speed_m_s));
     for loop=1:loops
      loiterAdded=false;
      for direction=[1,-1]
       loiter=d11_segment(pose,'arc',2*pi*task.minimumTurnRadius_m,task.minimumTurnRadius_m,direction,'vertical_acquisition',best);
       if d11_chart_clear(loiter,task.chart),append(loiter);loiterAdded=true;break;end
      end
      assert(loiterAdded,'Vertical preparation needs a clear forward loiter; no candidate found.');
     end
     [bestParts,bestAudit]=d11_connect(pose,[points(best,1:2)';heading],task.minimumTurnRadius_m,task.chart,'visit_approach',best);assert(bestAudit.accepted);
    end
   end
   append(bestParts);logConnection(bestAudit);lastDepth=points(best,3);order(end+1)=best;remaining(remaining==best)=[];
  end
  plan.targets=struct('kind','depth_visit_points','positionsNED',points,'visitOrder',order,...
   'captureRadius_m',spec.captureRadius_m,'maximumBoundDistance_m',spec.maximumBoundDistance_m,'depthTolerance_m',spec.depthTolerance_m,...
   'ordering','priority then shortest chart-clear Dubins connection from current pose; no claim of global TSP optimum');
end
connectTo(task.homePose(:),'return',0);
plan.totalLength_m=sum([plan.segments.length]);plan.geometricETA_s=plan.totalLength_m/task.speed_m_s;
plan.withinTimeBudget=plan.geometricETA_s<=task.maximumTime_s;
plan.allTargetsGeometricallyPlanned=isempty(plan.unplannedTargets);
plan.scope='Public structured task and known chart only. Geometric curvature/time screening; controller, sensed coverage, dynamic avoidance, navigation and vertical limits still require full MIL verification.';
 function append(parts)
  if isempty(parts),return;end
  if isempty(plan.segments),plan.segments=parts;else,plan.segments=[plan.segments,parts];end
  pose=[parts(end).finish;parts(end).heading1];
 end
 function logConnection(audit)
  if isempty(plan.connections),plan.connections=audit;else,plan.connections(end+1)=audit;end
 end
 function connectTo(goal,role,target)
  [parts,audit]=d11_connect(pose,goal,task.minimumTurnRadius_m,task.chart,role,target);
  assert(audit.accepted,'No chart-clear Dubins candidate found; not proof that all feasible paths are impossible.');append(parts);logConnection(audit);
 end
end
