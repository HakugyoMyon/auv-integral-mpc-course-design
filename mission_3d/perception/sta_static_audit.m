function audit=sta_static_audit(file,options)
% Independent archived packet/map admission and offline physical clearance audit.
% Does not expose truth to map/planner. Not a fresh stochastic autonomy trial.
if nargin<2,options=struct();end
if ~isfield(options,'physicalReplay'),options.physicalReplay=true;end
if ~isfield(options,'controlReplay'),options.controlReplay=false;end
fprintf('[static audit] Loading archive: %s\n',file);
z=load(file);checks=struct();checks.packetSchema=true;checks.captureTimes=true;
fprintf('[static audit] Archive loaded; map replay begins (%d gates, %d scans).\n',numel(z.staticGateLog),numel(z.scanLog));
map=sta_map_init(z.staticSpec.launchPrior,z.scanner);next=1;gateCount=0;
stateMismatch=0;acceptMismatch=0;admittedRejected=0;latestTime=-Inf;
for k=1:numel(z.scanLog)
 s=z.scanLog{k};p=s.packet;
 checks.packetSchema=checks.packetSchema&&~hasTruthField(p)&&~hasTruthField(s.captureNav);
 checks.captureTimes=checks.captureTimes&&abs(p.time-s.captureNav.time)<1e-6&&p.availableTime>=p.time&&p.time>latestTime;
 latestTime=p.time;
end
for k=1:numel(z.staticGateLog)
 if mod(k-1,100)==0,fprintf('[static audit] Map gate %d/%d; replayed scans %d.\n',k,numel(z.staticGateLog),next-1);end
 g=z.staticGateLog{k};
 while next<=numel(z.scanLog)&&z.scanLog{next}.packet.availableTime<=g.time+1e-8
  s=z.scanLog{next};[map,event]=sta_map_update(map,s.packet,s.captureNav);
  checks.captureTimes=checks.captureTimes&&event.accepted==s.event.accepted;next=next+1;
 end
 a=g.admission;q=z.planner.queryFcn(map,a.pointsNED,a.margin_m);gateCount=gateCount+1;
 stateMismatch=stateMismatch+sum(q.state~=a.state);
 acceptMismatch=acceptMismatch+double(all(q.free)~=a.accepted);
 % Only accepted execution gates were committed. Failed candidate admissions
 % are valid evidence of refusal and must not be counted as committed routes.
 executionAccepted=false;
 for j=1:numel(z.gateLog)
  e=z.gateLog{j};if abs(e.time-g.time)<1e-8,executionAccepted=e.gate.accepted;break;end
 end
 admittedRejected=admittedRejected+double(executionAccepted&&~all(q.free));
end
checks.admissionsReproduce=stateMismatch==0&&acceptMismatch==0;
checks.committedPrefixesObservedFree=admittedRejected==0;
fprintf('[static audit] Map replay finished; task consistency begins.\n');
[counts,falseClaims]=auv_task_truth_counts(z.plan,z.payloadLedger,z.h,z.payloadLog,z.payloadConfig);
checks.taskReportConsistent=sum(counts>=z.payloadLedger.minimumObservations)==z.report.true_target_confirmations&&falseClaims==z.report.false_confirmations;
checks.taskReportConsistent=checks.taskReportConsistent&&sum(z.payloadLedger.counts>=z.payloadLedger.minimumObservations)==z.report.confirmed_targets;
minimumClearance=Inf;minimumTerrain=Inf;maximumStateError=NaN;physicalSamples=0;
if options.physicalReplay
 fprintf('[static audit] Physics replay begins (%d archived control samples).\n',numel(z.h.t));
 x=z.h.truth(1,:)';maximumStateError=0;dt=z.c.dt;
 for k=1:numel(z.h.t)
  if mod(k-1,1000)==0,fprintf('[static audit] Physics control %d/%d; simulated t=%.1f s.\n',k,numel(z.h.t),z.h.t(k));end
  maximumStateError=max(maximumStateError,max(abs(x-z.h.truth(k,:)')));
  duration=z.h.appliedCommandDuration_s(k);steps=round(duration/dt);
  assert(abs(duration-steps*dt)<1e-7,'Nonintegral archived command duration.');
  command=[auv6_thrust_pwm(z.h.command(k,1),z.p);z.h.command(k,2:5)'];
  for j=1:steps
   minimumClearance=min(minimumClearance,sta_world_clearance(x,z.staticSpec.world));
   minimumTerrain=min(minimumTerrain,auv6_clearance_bound(x,@auv6_terrain));physicalSamples=physicalSamples+1;
   a=auv6_actuated_rhs(x,command,z.p,z.f);b=auv6_actuated_rhs(x+dt*a/2,command,z.p,z.f);
   d=auv6_actuated_rhs(x+dt*b/2,command,z.p,z.f);e=auv6_actuated_rhs(x+dt*d,command,z.p,z.f);
   x=x+dt/6*(a+2*b+2*d+e);x(4:7)=x(4:7)/norm(x(4:7));
  end
 end
 minimumClearance=min(minimumClearance,sta_world_clearance(x,z.staticSpec.world));
 minimumTerrain=min(minimumTerrain,auv6_clearance_bound(x,@auv6_terrain));
 maximumStateError=max(maximumStateError,max(abs(x-z.h.terminalPhysicalState)));
 checks.physicsReproduce=maximumStateError<1e-9;
 checks.actualBodyClearance=minimumClearance>=z.staticSpec.minimumClearance_m;
 checks.actualTerrainClearance=minimumTerrain>=2;
 % The independent audit includes terminal state, which the runtime observer
 % may not have sampled. Therefore report agreement is one-sided conservative.
 checks.clearanceReportNotOptimistic=z.report.minimum_obstacle_clearance_m<=minimumClearance+1e-7;
 fprintf('[static audit] Physics replay finished; maximum state error %.3g.\n',maximumStateError);
end
if options.controlReplay
 fprintf('[static audit] Independent control replay begins.\n');
 auditControl=auv_task_replay_control(file);checks.controlReplay=auditControl.passed;
 fprintf('[static audit] Independent control replay finished; passed=%d.\n',auditControl.passed);
end
values=struct2cell(checks);audit=struct('passed',all(cellfun(@(v)logical(v),values)),...
 'checks',checks,'replayed_map_gates',gateCount,'state_mismatches',stateMismatch,...
 'acceptance_mismatches',acceptMismatch,'committed_free_failures',admittedRejected,...
 'physical_samples',physicalSamples,'maximum_physical_state_error',maximumStateError,...
 'minimum_actual_obstacle_clearance_m',minimumClearance,'minimum_actual_terrain_clearance_m',minimumTerrain,...
 'physical_replay_enabled',options.physicalReplay,'control_replay_enabled',options.controlReplay,...
 'scope','Archived capture-time packet/schema/map admission replay and independent RK4 full-body offline clearance. Schema checking detects named truth fields, not a formal whole-program information-flow proof. Sampled 0.01s physical clearance is not continuous-time collision proof. Unexecuted options are explicitly marked; no fresh stochastic or real-world guarantee.');
fid=fopen(fullfile(fileparts(file),'static_audit.json'),'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true));
fprintf('[static audit] Finished; passed=%d; output static_audit.json.\n',audit.passed);
end

function found=hasTruthField(value)
found=false;if isstruct(value)
 names=fieldnames(value);bad={'truth','world','obstacles','centerNED','centerNE','radius_m','depthLimits_m','truthForOfflineAuditOnly'};
 if any(ismember(lower(string(names)),lower(string(bad)))),found=true;return;end
 for k=1:numel(value),for j=1:numel(names),if hasTruthField(value(k).(names{j})),found=true;return;end;end;end
elseif iscell(value),for k=1:numel(value),if hasTruthField(value{k}),found=true;return;end;end
end
end
