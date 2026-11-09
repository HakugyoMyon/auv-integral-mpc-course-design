function result=sta_planning_contract(outdir)
% Synthetic geometry contracts ONLY. No MIL or sensor qualification claim.
templates=auv_task_catalog();cfg=templates{3};cfg.task.visit.pointsNED=[120,0,14];cfg.task.visit.priority=1;cfg.task.maximumTime_s=1200;
[check,master]=auv_task_validate(cfg);assert(check.valid,strjoin(check.errors));
x=zeros(18,1);x(3)=14;x(4)=1;x(8)=.8;mission=struct('segment',1,'progress',0);
c=sta_planner_config();c.queryFcn=@syntheticQuery;c.navRadius95_m=.3;c.trackingTube_m=.2;c.attitudeBudget_rad=deg2rad(1);
free=struct('kind','free');p=sta_local_plan(x,master,mission,free,c);assert(p.accepted&&strcmp(p.route.maneuverKind,'nominal'));validateSegments(p.route);
blocked=struct('kind','occupied');p=sta_local_plan(x,master,mission,blocked,c);assert(~p.accepted);
unknown=struct('kind','unknown');p=sta_local_plan(x,master,mission,unknown,c);assert(~p.accepted);
limited=struct('kind','limited','northLimit_m',20);p=sta_local_plan(x,master,mission,limited,c);
assert(p.accepted&&p.commitDistance_m<=c.nominalSpeed_m_s*c.commitHorizon_s+1e-9);
assert(p.candidates{p.selectedIndex}.unknownSamples>0,'Unknown tail must remain explicitly uncommitted.');
sphere=struct('kind','sphere','centerNED',[50,0,14],'radius_m',1);
c.preferredAxis='horizontal';side=sta_local_plan(x,master,mission,sphere,c);
assert(side.accepted&&strcmp(side.route.maneuverKind,'horizontal'));
assert(~side.candidates{side.selectedIndex}.knownOccupiedCollision);validateSegments(side.route);
c.preferredAxis='vertical';vertical=sta_local_plan(x,master,mission,sphere,c);
assert(vertical.accepted&&strcmp(vertical.route.maneuverKind,'vertical'));
assert(abs(vertical.route.depthTargets(1)-c.nominalDepth_m)>3);
assert(vertical.route.depthTargets(end)==c.nominalDepth_m);
assert(~vertical.candidates{vertical.selectedIndex}.knownOccupiedCollision);validateSegments(vertical.route);
assert(vertical.route.depthSchedule(3,1)>=c.threatLookahead_m,'Depth restoration must begin only after position hold length.');
result=struct('passed',true,'cases',6,'horizontalSelectedSide',side.route.maneuverSide,...
 'verticalSelectedSide',vertical.route.maneuverSide,'maximumCommitDistance_m',p.commitDistance_m,...
 'scope','Synthetic complete-free/occupied/unknown/limited-free-prefix and known-sphere map contracts. No real sonar, occlusion, navigation, MPC, six-DOF or mission completion qualification.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'planning_contract.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
end
function q=syntheticQuery(map,p,margin)
n=size(p,1);state=zeros(n,1);clearance=inf(n,1);
switch map.kind
 case 'free'
 case 'occupied',state(:)=1;clearance(:)=-margin;
 case 'unknown',state(:)=-1;clearance(:)=-margin;
 case 'limited'
  clearance=map.northLimit_m-p(:,1)-margin;state(clearance<0)=-1;
 case 'sphere'
  clearance=vecnorm(p-map.centerNED,2,2)-map.radius_m-margin;state(clearance<=0)=1;
 otherwise,error('Unknown synthetic map.');
end
q=struct('state',state,'clearance',clearance);
end
function validateSegments(route)
segments=route.segments;
assert(numel(route.depthTargets)==numel(segments));
for k=1:numel(segments)
 assert(segments(k).length>=0&&strcmp(segments(k).role,'static_avoidance')&&segments(k).target==0);
 if strcmp(segments(k).kind,'arc'),assert(segments(k).radius>=25);end
 if k>1
  assert(norm(segments(k).start-segments(k-1).finish)<1e-6);
  assert(abs(atan2(sin(segments(k).heading0-segments(k-1).heading1),cos(segments(k).heading0-segments(k-1).heading1)))<1e-7);
 end
end
assert(all(diff(route.depthSchedule(:,1))>0));
end
