function result=sta_helical_contract(outdir)
% Synthetic primitive geometry only: no claim of sensor/loiter/MPC feasibility.
templates=auv_task_catalog();cfg=templates{3};cfg.task.visit.pointsNED=[250,0,14];
cfg.task.visit.priority=1;cfg.task.maximumTime_s=1800;
[check,master]=auv_task_validate(cfg);assert(check.valid,strjoin(check.errors));
x=zeros(18,1);x(3)=14;x(4)=1;x(8)=.8;mission=struct('segment',1,'progress',0);
c=sta_planner_config();c.queryFcn=@wallQuery;c.navRadius95_m=.3;c.trackingTube_m=.2;c.attitudeBudget_rad=deg2rad(1);
c.maximumRouteLength_m=360;c.enableHelical=true;
p=sta_local_plan(x,master,mission,struct('unknown',false),c);
assert(p.accepted&&strcmp(p.route.maneuverKind,'helical_climb_then_pass'));
r=p.route;a=p.candidates{p.selectedIndex};lead=r.segments(1);circle=r.segments(2);
assert(strcmp(lead.kind,'line')&&lead.length==15&&r.depthTargets(1)==14);
assert(strcmp(circle.kind,'arc')&&circle.radius==25&&abs(circle.length-2*pi*25)<1e-9);
assert(norm(circle.start-circle.finish)<1e-9);
assert(abs(atan2(sin(circle.heading1-circle.heading0),cos(circle.heading1-circle.heading0)))<1e-9);
assert(abs(r.depthTargets(2)-7.5)<1e-9&&r.depthTargets(3)==r.depthTargets(2));
assert(r.depthTargets(4)==14&&a.hold_m==50&&abs(a.turnDirection)==1);
assert(a.slewOkay&&a.depthOkay&&~a.knownOccupiedCollision&&r.totalLength_m<=360);
assert(isequal(r.originPointer,mission));
% The depth restore is position-triggered after a FULL circle and the pass.
assert(isequal(r.depthSchedule(1:2,:),[0,14;15,14]));
assert(r.depthSchedule(4,1)>=lead.length+circle.length+50-1e-9);
assert(max(r.depthSchedule(:,2))<=20&&min(r.depthSchedule(:,2))>=4);
c.enableHelical=false;off=sta_local_plan(x,master,mission,struct('unknown',false),c);
assert(~any(cellfun(@(q)strcmp(q.kind,'helical_climb_then_pass'),off.candidates)));
c.enableHelical=true;c.maximumRouteLength_m=260;
short=sta_local_plan(x,master,mission,struct('unknown',false),c);
assert(~any(cellfun(@(q)strcmp(q.kind,'helical_climb_then_pass'),short.candidates)));
c.maximumRouteLength_m=360;
unknown=sta_local_plan(x,master,mission,struct('unknown',true),c);assert(~unknown.accepted);
result=struct('passed',true,'cases',4,'selectedDirection',r.helicalTurnDirection,...
 'routeLength_m',r.totalLength_m,'scope','Synthetic helical primitive geometry, disabled compatibility, unchanged length cap and unknown-prefix contracts only. No sonar, known-free turn-circle, actuator, MPC or full-task qualification.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'helical_contract.json'),'w');assert(fid>=0);
 cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
end
end
function q=wallQuery(map,points,margin)
% A synthetic finite 3-D wall defeats short straight vertical and S routes.
delta=abs(points-[58.5,0,14])-[.5,100,4.5];
distance=vecnorm(max(delta,0),2,2)+min(max(delta,[],2),0);
clearance=distance-margin;state=zeros(size(clearance));state(clearance<=0)=1;
if map.unknown,state(:)=-1;clearance(:)=-Inf;end
q=struct('state',state,'clearance',clearance);
end
