function dyn_validate(kind,s)
t=dyn_contract(kind);assert(isequal(sort(fieldnames(s)),sort(fieldnames(t))),'Contract fields mismatch');assert(strcmp(s.schema,t.schema));
switch kind
 case 'task'
 assert(strcmp(s.frame,'NED') && any(strcmp(s.kind,{'survey','inspection','visits'})));
 assert(size(s.regionNE,2)==2 && size(s.targetsNED,2)==3 && s.maximumTime>0 && s.lineSpacing>0 && s.inspectionOffset>0 && all(isfinite(s.homeNED)));
 assert(numel(s.homeNED)==3 && all(isfinite(s.regionNE),'all') && all(isfinite(s.targetsNED),'all') && s.depth>0 && s.altitude>0);
 assert(isempty(s.priority) || (numel(s.priority)==size(s.targetsNED,1) && all(isfinite(s.priority))));
 assert(iscell(s.noGoRegionsNE));
 for k=1:numel(s.noGoRegionsNE),p=s.noGoRegionsNE{k};assert(size(p,1)>=3 && size(p,2)==2 && all(isfinite(p),'all'));end
 if strcmp(s.kind,'survey'),assert(size(s.regionNE,1)>=3,'Survey polygon missing');else,assert(~isempty(s.targetsNED),'Task targets missing');end
 case 'navigation'
 assert(strcmp(s.frame,'NED') && numel(s.positionNED)==3 && all(isfinite(s.positionNED)) && numel(s.quaternion)==4 && abs(norm(s.quaternion)-1)<1e-8);
 assert(isequal(size(s.positionCovNE),[2,2]) && all(isfinite(s.positionCovNE),'all') && norm(s.positionCovNE-s.positionCovNE','fro')<1e-10 && min(eig(s.positionCovNE))>=-1e-12);
 assert(isfinite(s.yawVariance) && s.yawVariance>=0 && isfinite(s.time) && isfinite(s.availableTime) && s.availableTime>=s.time);
 case 'trajectory'
 assert(strcmp(s.frame,'NED') && s.validUntil>=s.time && size(s.positionNED,2)==3 && numel(s.sampleTime)==size(s.positionNED,1) && numel(s.speed)==numel(s.sampleTime) && numel(s.heading)==numel(s.sampleTime));
 assert(all(isfinite(s.positionNED),'all') && all(isfinite(s.sampleTime)) && all(isfinite(s.speed)) && all(s.speed>=0) && all(isfinite(s.heading)));
 assert(all(s.sampleTime>=s.time) && all(s.sampleTime<=s.validUntil) && all(diff(s.sampleTime)>0));
 if s.feasibilityVerified,assert(~isempty(s.sampleTime) && all(diff(s.sampleTime)>0) && strcmp(s.status,'feasible'));end
 case 'ledger'
 assert(any(strcmp(s.status,{'pending','active','verified','skipped','failed'})));
 if strcmp(s.status,'verified'),assert(isfinite(s.verifiedAt)&&s.evidence.navigationValid&&s.evidence.observationValid&&isfinite(s.evidence.sampleTime)&&s.evidence.sampleTime<=s.verifiedAt);end
end
end
