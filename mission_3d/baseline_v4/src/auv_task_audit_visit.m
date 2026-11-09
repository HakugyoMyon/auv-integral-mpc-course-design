function result=auv_task_audit_visit(file)
% Replay ORIGINAL visit IDs from estimated state/uncertainty; truth only audits.
s=load(file,'h','c','plan','payloadLedger','mission','report');assert(auv_task_is_point_task(s.plan.task.type));
ledger=[];maximumError=0;trueSupport=[];
for k=1:numel(s.h.t)
 P=s.h.navigationCovariance(:,:,k);x=s.h.estimate(k,:)';R=auv6_rotation(x(4:7));
 state=struct('q',x(4:7),'P',P(1:15,1:15));quality=auv12b_nav_quality(state,s.c,s.h.t(k),s.h.t(k)-s.h.quality(k,3));
 H=zeros(3,16);H(:,1:3)=eye(3);H(:,7:9)=R*auv6_skew(s.c.rINS);bodyP=H*P*H';
 if ~isfield(s.h,'taskAcceptanceEnabled')||s.h.taskAcceptanceEnabled(k)
  ledger=auv_task_visit_ledger(ledger,s.plan,x,quality,sqrt(max(0,bodyP(3,3))),s.h.t(k));
 end
 assert(isequal(ledger.counts',s.h.taskCounts(k,:)),'Visit history differs from raw navigation replay.');
 maximumError=max(maximumError,norm(x(1:2)-s.h.truth(k,1:2)'));
end
assert(isequaln(ledger,s.payloadLedger));
for k=1:numel(ledger.events)
 e=ledger.events(k);[delta,row]=min(abs(s.h.t-e.time));assert(delta<1e-8);id=e.indices;
 target=s.plan.targets.positionsNED(id,:);truth=s.h.truth(row,1:3);
 supported=norm(truth(1:2)-target(1:2))<=s.plan.targets.captureRadius_m&&abs(truth(3)-target(3))<=s.plan.targets.depthTolerance_m;
 assert(supported,'Recorded visit lacks actual position/depth support.');trueSupport(end+1)=id;
end
result=struct('passed',true,'replayed_samples',numel(s.h.t),'actualSupportedVisitIds',trueSupport,...
 'onboardComplete',ledger.complete,'actualReturn',s.report.return_verified,'maximumObservedHorizontalNavError_m',maximumError,...
 'scope','Full original-ID visit ledger replay from saved navigation/covariance with actual geometry at each event. Does not replace raw navigation/physics replay or fresh independent trial.');
fid=fopen(fullfile(fileparts(file),'visit_ledger_audit.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
