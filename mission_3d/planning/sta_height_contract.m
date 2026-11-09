function result=sta_height_contract()
% Synthetic altitude_inheritance_v1 variant tests; no prior MIL run is relabeled.
c=sta_planner_config();s=d11_segment([0;0;0],'line',100,Inf,0,'work',1);
depth=struct('workMode','altitude','workAltitude_m',8,'workDepthBounds_m',[4,24]);
master=struct('segments',s,'task',struct('type','inspection','depth',depth));
p=struct('segment',1,'progress',0);
[inherit,bounds]=sta_altitude_inheritance(master,p,'horizontal',c);assert(inherit&&isequal(bounds,[4,20]));
r=master;r.staticAvoidance=true;r.depthTargets=14;r.inheritAltitudeMode=inherit;r.altitudeDepthBounds_m=bounds;
x=zeros(18,1);x(1)=10;x(3)=16;x(4)=1;q=struct('valid',true);alt=struct('valid',true,'time',1,'bottomDepth',24);
ledger=struct('complete',false,'counts',[0;1]);original=ledger;
[m,status]=sta_step_height([],r,x,q,.05,1,alt,ledger);
assert(status.workMode&&status.depthTarget==16&&~status.heightMissing&&~status.outsideDepthEnvelope);
[~,stale]=sta_step_height(m,r,x,q,.05,2,alt,ledger);assert(stale.heightMissing);
returnMaster=master;returnMaster.segments.role='return';
assert(~sta_altitude_inheritance(returnMaster,p,'horizontal',c));
point=master;point.task.type='visit';point.task.depth.workMode='depth';
assert(~sta_altitude_inheritance(point,p,'horizontal',c));
assert(~sta_altitude_inheritance(master,p,'helical_climb_then_pass',c));
assert(~sta_altitude_inheritance(master,p,'vertical',c));
bad=alt;bad.bottomDepth=29;[~,outside]=sta_step_height([],r,x,q,.05,1,bad,ledger);
assert(outside.outsideDepthEnvelope&&outside.depthTarget==21);
ledger.complete=true;[~,latched]=sta_step_height([],r,x,q,.05,1,alt,ledger);
assert(latched.workMode&&latched.depthTarget==16);ledger.complete=false;assert(isequaln(ledger,original));
tactical=r;tactical.inheritAltitudeMode=false;tactical.depthTargets=7.5;
[cache,t]=sta_step_height([],tactical,x,q,.05,1,alt,ledger);
assert(~t.workMode&&t.depthTarget==7.5&&cache.bottomDepth==24&&cache.lastBottomTime==1);
result=struct('passed',true,'checks',8,'executionVariant','altitude_inheritance_v1','scope','New separately selected variant inheritance, stale height, return/point/tactical exclusions, depth intersection, latched mode and bottom cache tests. No legacy entry-point changes, MIL or actual payload qualification.');
end
