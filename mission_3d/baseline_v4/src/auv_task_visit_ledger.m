function ledger=auv_task_visit_ledger(ledger,master,xhat,quality,depthSigma,time)
% Persistent ORIGINAL target IDs; geometry replanning never resets this ledger.
% Visit means bounded fly-through, not hover, docking or sensor inspection.
assert(auv_task_is_point_task(master.task.type)&&strcmp(master.targets.kind,'depth_visit_points'));
targets=master.targets;count=size(targets.positionsNED,1);
if isempty(ledger)
 ledger=struct('kind','depth_visit_points','counts',zeros(count,1),'minimumObservations',1,...
  'lastSampleTime',-Inf,'events',struct('time',{},'indices',{},'positionNED',{},'radius95',{},'depthBound',{}),...
  'fraction',0,'complete',false,'originalVisitOrder',targets.visitOrder(:)');
end
assert(numel(ledger.counts)==count&&isequal(ledger.originalVisitOrder,targets.visitOrder(:)'));
if time<=ledger.lastSampleTime||~quality.valid||~isfinite(depthSigma)||depthSigma<0||...
 ~isfinite(quality.radius95)||quality.radius95<0||numel(xhat)<3||any(~isfinite(xhat(1:3))),return;end
ledger.lastSampleTime=time;
remaining=ledger.originalVisitOrder(ledger.counts(ledger.originalVisitOrder)<1);
if isempty(remaining),return;end
index=remaining(1);goal=targets.positionsNED(index,:)';
distance=norm(xhat(1:2)-goal(1:2));depthBound=abs(xhat(3)-goal(3))+3*depthSigma+.05;
if distance+quality.radius95<=targets.captureRadius_m&&distance+quality.radius95<=targets.maximumBoundDistance_m&&depthBound<=targets.depthTolerance_m
 ledger.counts(index)=1;
 ledger.events(end+1)=struct('time',time,'indices',index,'positionNED',xhat(1:3),...
  'radius95',quality.radius95,'depthBound',depthBound);
end
ledger.fraction=mean(ledger.counts>=1);ledger.complete=all(ledger.counts>=1);
end
