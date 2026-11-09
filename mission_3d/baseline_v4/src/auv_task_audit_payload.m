function result=auv_task_audit_payload(file)
% Replay received payload/altimeter packets and first-confirmation support.
s=load(file,'h','plan','payloadLedger','payloadLog','payloadConfig','altimeterLog','altimeterConfig','c','report');
t=s.plan.task;assert(any(strcmp(t.type,{'survey','inspection'})));
if strcmp(t.type,'survey'),centers=s.plan.targets.centersNE;radius=sqrt(2)*s.plan.targets.cellSize_m/2;bearing=t.survey.bearing_rad;
else,centers=s.plan.targets.positionsNE;radius=0;bearing=0;end
ledger=d11_payload_ledger([],s.plan,struct('valid',false),struct('valid',false));trueCounts=zeros(size(ledger.counts));altitude=struct('valid',false,'time',-Inf,'bottomDepth',NaN);cursor=1;firstTimes=nan(size(trueCounts));
for k=1:numel(s.payloadLog)
 e=s.payloadLog{k};sampleRow=round(e.packet.time/.1)+1;row=round(e.acceptedAt/.1)+1;
 assert(max(abs(e.navigation-s.h.estimate(sampleRow,:)'))<1e-10);
 assert(max(abs(e.truthForOfflineAuditOnly-s.h.truth(sampleRow,:)'))<1e-10);
 assert(max(abs(e.attitudeCovariance-s.h.navigationCovariance(7:9,7:9,sampleRow)),[],'all')<1e-12);
 while cursor<=numel(s.altimeterLog)&&s.altimeterLog{cursor}.packet.availableTime<=e.acceptedAt+1e-9
  a=s.altimeterLog{cursor};ar=round(a.packet.time/.1)+1;assert(max(abs(a.navigation-s.h.estimate(ar,:)'))<1e-10);
  processed=auv6_altitude_estimate(a.packet,a.navigation(4:7),s.altimeterConfig);
  if processed.valid,altitude=struct('valid',true,'time',a.packet.time,'bottomDepth',a.navigation(3)+processed.altitude);end
  cursor=cursor+1;
 end
 assert(isequaln(altitude,e.heightEvidence));
 sampleQ=qualityAt(sampleRow);currentQ=qualityAt(row);
 obs=d11_payload_observe(e.packet,e.navigation,sampleQ,e.attitudeCovariance,centers,radius,s.payloadConfig,e.acceptedAt);
 if ~currentQ.valid||~altitude.valid||e.acceptedAt-altitude.time>.7||abs(altitude.bottomDepth-e.navigation(3)-t.depth.workAltitude_m)>.75,obs.valid=false;obs.indices=[];end
 assert(isequal(obs.indices,e.observation.indices)&&obs.valid==e.observation.valid);
 before=ledger.counts;
 if ~isfield(e,'taskAcceptanceEnabled')||e.taskAcceptanceEnabled,ledger=d11_payload_ledger(ledger,s.plan,obs,currentQ);else,obs.indices=[];end
 ids=obs.indices;truth=s.h.truth(sampleRow,:)';good=d12_payload_truth_check(ids,centers,radius,truth,s.payloadConfig,@auv6_terrain,bearing);
 good=good&abs(auv6_terrain(truth(1),truth(2))-truth(3)-t.depth.workAltitude_m)<=1;trueCounts(ids(good))=trueCounts(ids(good))+1;
 newly=find(before<ledger.minimumObservations&ledger.counts>=ledger.minimumObservations);assert(all(trueCounts(newly)>=ledger.minimumObservations),'Premature payload confirmation.');firstTimes(newly)=e.acceptedAt;
end
assert(isequaln(ledger,s.payloadLedger));
result=struct('passed',true,'replayedPackets',numel(s.payloadLog),'confirmedTargets',sum(ledger.counts>=ledger.minimumObservations),'firstConfirmationTimes',firstTimes,...
 'scope','Raw received payload/altimeter and archived navigation replay; actual geometric support at each first confirmation. No real imaging or defect recognition qualification.');
fid=fopen(fullfile(fileparts(file),'payload_replay_audit.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
 function q=qualityAt(index)
  state=struct('q',s.h.estimate(index,4:7)','P',s.h.navigationCovariance(1:15,1:15,index));q=auv12b_nav_quality(state,s.c,s.h.t(index),s.h.t(index)-s.h.quality(index,3));
 end
end
