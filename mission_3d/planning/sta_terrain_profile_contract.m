function result=sta_terrain_profile_contract()
% Nominal measured-plane geometry contracts only; no executed safety claim.
c=sta_planner_config();c.referenceDepth_m=14;c.nominalDepth_m=14;c.currentTime_s=1;
c.terrainPlane=struct('valid',true,'time',1,'gradientNE',[.04;0]);
s=(0:.5:100)';xy=[s,zeros(size(s))];x=zeros(18,1);x(3)=14;x(4)=1;
[z,sl,sc,meta]=sta_altitude_profile(xy,s,x,c);
assert(meta.planeUsed&&meta.accepted&&abs(z(end)-18)<1e-10&&max(abs(sl))<=.04+1e-10&&isequal(sc,[s,z]));
old=c;old.currentTime_s=2;[flat,~,~,stale]=sta_altitude_profile(xy,s,x,old);
assert(~stale.planeUsed&&all(flat==14));
limited=c;limited.maximumDepthRate_m_s=.02;
[slow,~,~,~]=sta_altitude_profile(xy,s,x,limited);
assert(all(abs(diff(slow))<=.02/.85*diff(s)+1e-12));
deep=c;deep.terrainPlane.gradientNE=[.07;0];[~,~,~,bad]=sta_altitude_profile(xy,s,x,deep);assert(~bad.withinDepthBounds&&~bad.accepted);
seg=d11_segment([0;0;0],'line',100,Inf,0,'static_avoidance',0);
r=struct('segments',seg,'staticAvoidance',true,'inheritAltitudeMode',true,'depthSchedule',[0,14;100,14]);
c.queryFcn=@freeQuery;pointer=struct('segment',1,'progress',0);
probe=sta_probe_terrain(r,pointer,x,14,14,struct(),c);
assert(probe.altitudeProfileMeta.planeUsed&&abs(probe.sampleDepth_m(end)-16.2)<1e-9);
templates=auv_task_catalog();cfg=templates{2};[check,master]=auv_task_validate(cfg);assert(check.valid,strjoin(check.errors));
index=find(~ismember({master.segments.role},{'departure','return'}),1);assert(~isempty(index));
pointer=struct('segment',index,'progress',0);x(1:2)=master.segments(index).start;
candidate=sta_local_plan_terrain(x,master,pointer,struct(),c);
assert(candidate.accepted&&candidate.route.inheritAltitudeMode&&isfield(candidate.route,'altitudeProfileMeta'));
assert(strcmp(candidate.executionVariant,'altitude_plane_nominal_v2'));
result=struct('passed',true,'checks',6,'executionVariant','altitude_plane_nominal_v2',...
 'scope','Fresh/stale plane, slew limit, declared depth rejection, probe profile and synthetic free-map candidate wiring only. Local-plane extrapolation is not global terrain truth or a whole-tail safety guarantee.');
end
function q=freeQuery(~,points,~)
q=struct('state',zeros(size(points,1),1),'clearance',inf(size(points,1),1));
end
