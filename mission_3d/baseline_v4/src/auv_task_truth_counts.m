function [counts,falseClaims]=auv_task_truth_counts(plan,ledger,h,payloadLog,payloadConfig)
% Offline support check only; never called for control or task decisions.
counts=zeros(size(ledger.counts));t=plan.task;
if auv_task_is_point_task(t.type)
 for k=1:numel(ledger.events)
  e=ledger.events(k);[gap,row]=min(abs(h.t-e.time));assert(gap<1e-8);
  id=e.indices;x=h.truth(row,:)';goal=plan.targets.positionsNED(id,:)';
  if norm(x(1:2)-goal(1:2))<=plan.targets.captureRadius_m&&abs(x(3)-goal(3))<=plan.targets.depthTolerance_m,counts(id)=counts(id)+1;end
 end
else
 if strcmp(t.type,'survey'),centers=plan.targets.centersNE;radius=sqrt(2)*plan.targets.cellSize_m/2;bearing=t.survey.bearing_rad;
 else,centers=plan.targets.positionsNE;radius=0;bearing=0;end
 for k=1:numel(payloadLog)
  e=payloadLog{k};if isfield(e,'taskAcceptanceEnabled')&&~e.taskAcceptanceEnabled,continue;end;ids=e.observation.indices;x=e.truthForOfflineAuditOnly;
  good=d12_payload_truth_check(ids,centers,radius,x,payloadConfig,@auv6_terrain,bearing);
  good=good&abs(auv6_terrain(x(1),x(2))-x(3)-t.depth.workAltitude_m)<=1;
  counts(ids(good))=counts(ids(good))+1;
 end
end
falseClaims=sum(ledger.counts>=ledger.minimumObservations&counts<ledger.minimumObservations);
end
