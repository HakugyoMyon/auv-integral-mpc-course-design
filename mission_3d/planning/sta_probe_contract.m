function result=sta_probe_contract(outdir)
% Budget/schedule agreement only, not a whole-task safety qualification.
c=sta_planner_config();c.queryFcn=@freeQuery;
seg=d11_segment([0;0;0],'line',100,Inf,0,'static_avoidance',0);
r=struct('segments',seg,'staticAvoidance',true,'depthSchedule',[0,14;100,14]);
m=struct('segment',1,'progress',0);x=zeros(18,1);x(3)=14;x(4)=1;
a=sta_route_check(r,struct(),c);b=sta_probe_consistent(r,m,x,14,14,struct(),c);
assert(abs(a.queryMargin_m-b.margin_m)<1e-12&&~b.knownObstacleAhead);
r.depthSchedule=[0,14;50,11;100,11];m.progress=20;
a=sta_route_check(r,struct(),c);b=sta_probe_consistent(r,m,x,14,14,struct(),c);
assert(abs(a.queryMargin_m-b.margin_m)<1e-12);
assert(abs(b.sampleDepth_m(1)-12.8)<1e-12&&abs(b.sampleDepthSlope(1)+.06)<1e-12);
assert(any(abs(b.sampleDistance_m-30)<1e-12)&&b.scheduledDepth);
ordinary=rmfield(r,{'staticAvoidance','depthSchedule'});
d=sta_probe_consistent(ordinary,m,x,14,11,struct(),c);
assert(~d.scheduledDepth&&d.sampleDepth_m(1)==14);
assert(max(abs(d.sampleDepthSlope))*c.maximumSpeed_m_s<=c.maximumDepthRate_m_s+1e-12);
result=struct('passed',true,'cases',3,'scope','Flat/sloped static margin agreement and ordinary bounded depth ramp only. Actual MPC trajectory admission remains separate.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'probe_contract.json'),'w');assert(fid>=0);
 cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
end
end
function q=freeQuery(~,points,~)
q=struct('state',zeros(size(points,1),1),'clearance',inf(size(points,1),1));
end
