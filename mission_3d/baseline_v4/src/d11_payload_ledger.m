function ledger=d11_payload_ledger(ledger,plan,observation,taskQuality)
% Coverage and facility observation counts, never route-gate substitutes.
if isempty(ledger)
 if strcmp(plan.targets.kind,'coverage_cells'),count=size(plan.targets.centersNE,1);else,count=size(plan.targets.positionsNE,1);end
 ledger=struct('counts',zeros(count,1),'lastSampleTime',-Inf,'events',struct('time',{},'indices',{}),...
  'fraction',0,'complete',false,'minimumObservations',plan.targets.minimumObservations);
end
if ~observation.valid||~taskQuality.valid||observation.time<=ledger.lastSampleTime,return;end
assert(all(observation.indices>=1&observation.indices<=numel(ledger.counts)));
ledger.lastSampleTime=observation.time;indices=unique(observation.indices);
ledger.counts(indices)=ledger.counts(indices)+1;
if ~isempty(indices),ledger.events(end+1)=struct('time',observation.time,'indices',indices);end
ledger.fraction=mean(ledger.counts>=ledger.minimumObservations);
if strcmp(plan.targets.kind,'coverage_cells'),ledger.complete=ledger.fraction>=plan.targets.requiredFraction;
else,ledger.complete=all(ledger.counts>=ledger.minimumObservations);end
end
