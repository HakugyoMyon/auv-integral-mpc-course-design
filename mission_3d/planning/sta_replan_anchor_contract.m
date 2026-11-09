function result=sta_replan_anchor_contract(outdir)
% Pure geometry and memory contracts, no sensor/MPC/whole-task claim.
line=d11_segment([0;0;0],'line',250,Inf,0,'transit',0);
master=struct('segments',line,'marker','original');
m=struct('segment',1,'progress',20,'filteredSideslip',.12,...
 'finished',true,'complete',true,'returnVerified',true,'terminalVerified',true,...
 'terminalApproachActive',true,'bottomDepth',27,'lastBottomTime',3);
[r,p,a]=sta_replan_anchor(master,m,pose(35,0,0));
assert(isequaln(r,master)&&a.accepted&&a.projectionAccepted&&p.progress==35);
assert(p.filteredSideslip==m.filteredSideslip&&p.bottomDepth==27&&p.lastBottomTime==3);
assert(~p.finished&&~p.complete&&~p.returnVerified&&~p.terminalVerified&&~p.terminalApproachActive);
local=struct('staticAvoidance',true,'resumeRoute',master,'resumePoint',struct('segment',1,'progress',100));
[r,p,a]=sta_replan_anchor(local,m,pose(45,8,0));
assert(isequaln(r,master)&&a.accepted&&~a.projectionAccepted&&p.progress==100&&a.unwrappedDetours==1);
[~,p,a]=sta_replan_anchor(local,m,pose(110,2,0));assert(a.projectionAccepted&&p.progress==110);
[~,p,a]=sta_replan_anchor(local,m,pose(105,0,pi));assert(~a.projectionAccepted&&p.progress==100);
[~,p,a]=sta_replan_anchor(local,m,pose(280,0,0));assert(~a.projectionAccepted&&p.progress==100);
nested=struct('staticAvoidance',true,'resumeRoute',local,'resumePoint',struct('segment',1,'progress',5));
[r,p,a]=sta_replan_anchor(nested,m,pose(110,0,0));assert(isequaln(r,master)&&a.unwrappedDetours==2&&p.progress==110);
assert(m.progress==20&&m.finished&&m.filteredSideslip==.12);
% Two fresh replans may update the estimated route projection, not consume
% the unachieved future join and then add another join advance on each replan.
local.originPointer=struct('segment',1,'progress',20);
local.resumePoint.progress=90;
[~,p,a]=sta_replan_anchor(local,m,pose(45,-9,0));
assert(a.projectionAccepted&&~a.legacyFutureAnchor&&abs(p.progress-45)<1e-9);
again=local;again.originPointer=struct('segment',p.segment,'progress',p.progress);
again.resumePoint.progress=p.progress+90;
[~,p2,a2]=sta_replan_anchor(again,m,pose(54,-8.5,0));
assert(a2.projectionAccepted&&abs(p2.progress-54)<1e-9&&p2.progress<again.resumePoint.progress);
[~,p3,a3]=sta_replan_anchor(local,m,pose(45,30,0));
assert(~a3.projectionAccepted&&p3.progress==20&&~a3.legacyFutureAnchor);
result=struct('passed',true,'cases',8,...
 'scope','Synthetic normal/local/legacy/heading/window/nested and repeated-replanning origin contracts. No ledger or physical execution mutation; future join is not achieved progress.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'replan_anchor_contract.json'),'w');assert(fid>=0);
 cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
end
end
function x=pose(n,e,yaw)
x=zeros(18,1);x(1:3)=[n;e;14];x(4:7)=[cos(yaw/2);0;0;sin(yaw/2)];x(8)=.8;
end

