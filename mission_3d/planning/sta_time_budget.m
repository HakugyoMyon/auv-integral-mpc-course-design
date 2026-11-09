function audit=sta_time_budget(detour,ledger,elapsed_s,maximumTime_s)
% Conservative nominal time screen, NOT a speed/energy/execution guarantee.
% Pending work skipped by the rejoin is UNKNOWN, never silently costed zero.
assert(isfinite(elapsed_s)&&elapsed_s>=0&&isfinite(maximumTime_s)&&maximumTime_s>0);
assert(isfield(detour,'resumeRoute')&&isfield(detour,'resumePoint'));
master=detour.resumeRoute;anchor=detour.resumePoint;segments=master.segments;
assert(anchor.segment>=1&&anchor.segment<=numel(segments));
assert(anchor.progress>=0&&anchor.progress<=segments(anchor.segment).length+1e-8);
detourLength=sum([detour.segments.length]);
remainingLength=max(0,segments(anchor.segment).length-anchor.progress)+sum([segments(anchor.segment+1:end).length]);
speed=.75;reserve=15;required=(detourLength+remainingLength)/speed+reserve;
remainingTime=maximumTime_s-elapsed_s;
missing=[];unknownReason='';
if isempty(ledger)||~isfield(ledger,'complete')
 unknownReason='missing_original_task_ledger';
elseif ~ledger.complete
 if ~isfield(master,'targets')||~isfield(master.targets,'positionsNED')||...
   ~isfield(ledger,'originalVisitOrder')||numel(ledger.counts)~=size(master.targets.positionsNED,1)||...
   ~isfield(master.targets,'visitOrder')||~isequal(ledger.originalVisitOrder(:),master.targets.visitOrder(:))
  unknownReason='payload_or_original_identity_mapping_not_proven';
 else
  pending=find(ledger.counts<ledger.minimumObservations);
  future=segments(anchor.segment:end);ids=[future.target];
  % A rejoin at the current segment's endpoint has already omitted its work.
  if segments(anchor.segment).length-anchor.progress<=.8,ids=ids(2:end);end
  missing=pending(~ismember(pending,ids));
  if ~isempty(missing),unknownReason='unconfirmed_targets_omitted_by_rejoin_require_uncosted_recovery';end
 end
end
status='pass';reason='nominal_route_and_remaining_work_time_fit';
if ~isempty(unknownReason),status='unknown';reason=unknownReason;
elseif required>remainingTime,status='insufficient';reason='nominal_route_plus_reserve_exceeds_remaining_time';end
audit=struct('status',status,'accepted',strcmp(status,'pass'),'reason',reason,...
 'detourLength_m',detourLength,'remainingMasterLength_m',remainingLength,...
 'assumedMinimumProgressSpeed_m_s',speed,'fixedReserve_s',reserve,...
 'requiredNominalTime_s',required,'remainingTime_s',remainingTime,...
 'nominalTimeFits',required<=remainingTime,'uncostedPendingTargetIDs',missing,...
 'scope','Route-length time screen at assumed0.75m/s plus15s. Unknown if original-ID mapping or skipped pending-work recovery cost is unproven. Does not guarantee actual minimum progress speed, scan availability, future replans, payload dwell, physical return, or battery endurance. Diagnostic only; do not present unknown as time-admissible.');
end
