function plan=auv_task_plan(task)
% Existing work-task geometry plus explicit destination/home operations.
if ~any(strcmp(task.type,{'transit','return'})),plan=d42_task_plan(task);plan=auv_task_vertical_screen(plan,task.depth.transit_m);return;end
assert(strcmp(task.schema,'auv.actual_task.v1')&&isequal(task.homeReturnHeadingRequired,false));
assert(task.speed_m_s==.8&&task.minimumTurnRadius_m>=25&&task.departureLength_m>=8&&task.chart.clearance_m>=7);
assert(task.depth.maximumRate_m_s>0&&task.depth.maximumRate_m_s<=.06);
if strcmp(task.type,'transit'),spec=task.transit;goal=reshape(spec.destinationNED,3,1);kind='destination';
else,spec=struct('captureRadius_m',3,'maximumBoundDistance_m',5,'depthTolerance_m',.5);goal=[task.homePose(1:2)';task.depth.return_m];kind='home';end
assert(all(isfinite(goal))&&goal(3)>=4&&spec.captureRadius_m>0&&spec.maximumBoundDistance_m>=spec.captureRadius_m&&spec.depthTolerance_m>0);
assert(d11_chart_clear(d11_segment([goal(1:2);0],'line',0,Inf,0),task.chart),'Endpoint is outside the declared chart clearance.');
pose=task.startPose(:);departure=d11_segment(pose,'line',task.departureLength_m,Inf,0,'departure',0);assert(d11_chart_clear(departure,task.chart));
pose=[departure.finish;departure.heading1];bearing=atan2(goal(2)-pose(2),goal(1)-pose(1));
headings=unique([bearing,0,pi/2,pi,-pi/2]);required=(abs(goal(3)-task.depth.transit_m)/task.depth.maximumRate_m_s+5)*.85;
best=[];bestAudit=[];length=Inf;role='transit';if strcmp(task.type,'return'),role='return';end
for heading=headings
 [parts,audit]=d15_connect(pose,[goal(1:2);heading],task.minimumTurnRadius_m,task.chart,role,1,required);
 if audit.accepted&&~isempty(parts)&&audit.length_m<length,best=parts;bestAudit=audit;length=audit.length_m;end
end
assert(~isempty(best),'No finite chart-clear candidate supports endpoint and vertical travel time. Not a proof of global infeasibility.');
segments=[departure,best];targets=struct('kind','depth_visit_points','positionsNED',goal','visitOrder',1,'captureRadius_m',spec.captureRadius_m,...
 'maximumBoundDistance_m',spec.maximumBoundDistance_m,'depthTolerance_m',spec.depthTolerance_m,'ordering','single operation endpoint');
plan=struct('task',task,'segments',segments,'connections',bestAudit,'unplannedTargets',[],'targets',targets,'totalLength_m',sum([segments.length]),...
 'geometricETA_s',sum([segments.length])/task.speed_m_s,'withinTimeBudget',sum([segments.length])/task.speed_m_s<=task.maximumTime_s,...
 'allTargetsGeometricallyPlanned',true,'terminalGoalNED',goal,'terminalKind',kind,...
 'scope','Explicit fly-through endpoint, not docking or stopping. Nominal vertical time uses .85m/s upper branch and5s reserve; MIL verification remains necessary.');
plan=auv_task_vertical_screen(plan,task.depth.transit_m);
end
