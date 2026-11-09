function result=sta_time_budget_contract(outdir)
% Pure bookkeeping/time arithmetic; does not run MATLAB dynamics or sensors.
work=d11_segment([0;0;0],'line',100,Inf,0,'visit',1);
home=d11_segment([100;0;0],'line',100,Inf,0,'return',0);
master=struct('segments',[work,home],'targets',struct('positionsNED',[100,0,14],'visitOrder',1));
detour=struct('segments',d11_segment([0;0;0],'line',120,Inf,0,'static_avoidance',0),...
 'resumeRoute',master,'resumePoint',struct('segment',1,'progress',50));
ledger=struct('complete',false,'counts',0,'minimumObservations',1,'originalVisitOrder',1);
a=sta_time_budget(detour,ledger,20,1000);
assert(a.accepted&&strcmp(a.status,'pass')&&abs(a.requiredNominalTime_s-(270/.75+15))<1e-9);
b=sta_time_budget(detour,ledger,900,1000);assert(~b.accepted&&strcmp(b.status,'insufficient'));
detour.resumePoint=struct('segment',2,'progress',0);
c=sta_time_budget(detour,ledger,20,1000);
assert(~c.accepted&&strcmp(c.status,'unknown')&&isequal(c.uncostedPendingTargetIDs,1));
ledger.complete=true;ledger.counts=1;d=sta_time_budget(detour,ledger,20,1000);assert(d.accepted);
result=struct('passed',true,'cases',4,'scope','Synthetic route-length arithmetic, insufficient time, skipped-original-target unknown and complete-ledger cases only.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'time_budget_contract.json'),'w');assert(fid>=0);
 cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
end
end
