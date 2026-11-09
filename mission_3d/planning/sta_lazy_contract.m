function result=sta_lazy_contract(outdir)
% Hierarchical catalog policy contracts; not a performance/whole-mission claim.
templates=auv_task_catalog();cfg=templates{3};cfg.task.visit.pointsNED=[250,0,14];
cfg.task.visit.priority=1;cfg.task.maximumTime_s=1800;
[check,master]=auv_task_validate(cfg);assert(check.valid,strjoin(check.errors));
x=zeros(18,1);x(3)=14;x(4)=1;x(8)=.8;mission=struct('segment',1,'progress',0);
c=sta_planner_config();c.enableHelical=true;c.maximumRouteLength_m=360;
c.queryFcn=@query;c.navRadius95_m=.3;c.trackingTube_m=.2;c.attitudeBudget_rad=deg2rad(1);
free=sta_local_plan_lazy(x,master,mission,struct('wall',false),c);
assert(free.accepted&&free.standardFeasible&&~free.helicalCatalogEvaluated);
assert(~any(cellfun(@(a)strcmp(a.kind,'helical_climb_then_pass'),free.candidates)));
wall=sta_local_plan_lazy(x,master,mission,struct('wall',true),c);
assert(wall.accepted&&~wall.standardFeasible&&wall.helicalCatalogEvaluated);
assert(strcmp(wall.route.maneuverKind,'helical_climb_then_pass'));
result=struct('passed',true,'cases',2,'scope','Free map skips helical evaluation; synthetic wall triggers helical fallback. Hierarchical feasibility, not global candidate cost optimality or timed runtime proof.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'lazy_contract.json'),'w');assert(fid>=0);
 cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
end
end
function q=query(map,points,margin)
state=zeros(size(points,1),1);clearance=inf(size(state));
if map.wall
 delta=abs(points-[58.5,0,14])-[.5,100,4.5];
 clearance=vecnorm(max(delta,0),2,2)+min(max(delta,[],2),0)-margin;
 state(clearance<=0)=1;
end
q=struct('state',state,'clearance',clearance);
end
