function audit=sta_nav_envelope_audit(file)
% Empirical archived pose-bound check at actual mapping and admission times.
% Not calibration of statistical confidence or a continuous-time guarantee.
z=load(file,'h','scanLog','staticGateLog');h=z.h;
assert(all(diff(h.t)>0)&&size(h.truth,1)==numel(h.t));
scanRecords={};for k=1:numel(z.scanLog)
 e=z.scanLog{k};if e.event.accepted,scanRecords{end+1}=e.captureNav;end %#ok<AGROW>
end
gateRecords=cell(size(z.staticGateLog));for k=1:numel(gateRecords),gateRecords{k}=z.staticGateLog{k}.navigation;end
scan=evaluate(scanRecords,h);gate=evaluate(gateRecords,h);
audit=struct('passed',scan.passed&&gate.passed,'acceptedScanCapturePose',scan,'staticGatePose',gate,...
 'scope','Empirical check of archived body position and full quaternion angle against the exact bound on each accepted scan capture and each static gate. Missing truth-time association or malformed pose/bound fails. No confidence calibration, unseen-condition guarantee or between-sample/continuous-time bound claim. Conditional set-fusion arguments require these assumptions independently; this finite audit alone is not their proof.');
fid=fopen(fullfile(fileparts(file),'navigation_envelope_audit.json'),'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true));disp(audit);
end

function result=evaluate(records,h)
rows=nan(numel(records),7);missing=[];malformed=[];
for k=1:numel(records)
 n=records{k};required={'time','positionNED','quaternion','positionErrorBound_m','attitudeErrorBound_rad'};
 if ~all(isfield(n,required)),malformed(end+1)=k;continue;end %#ok<AGROW>
 if ~isscalar(n.time)||~isfinite(n.time)||~isscalar(n.positionErrorBound_m)||~isscalar(n.attitudeErrorBound_rad)
  malformed(end+1)=k;continue; %#ok<AGROW>
 end
 [gap,id]=min(abs(h.t-n.time));
 if gap>1e-7,missing(end+1)=n.time;continue;end %#ok<AGROW>
 q=n.quaternion(:);truth=h.truth(id,4:7)';p=n.positionNED(:);
 if numel(q)~=4||numel(p)~=3||any(~isfinite([q;p;n.positionErrorBound_m;n.attitudeErrorBound_rad]))||...
   norm(q)<eps||norm(truth)<eps||n.positionErrorBound_m<0||n.attitudeErrorBound_rad<0
  malformed(end+1)=k;continue; %#ok<AGROW>
 end
 q=q/norm(q);truth=truth/norm(truth);angle=2*acos(min(1,max(0,abs(dot(q,truth)))));
 position=norm(p-h.truth(id,1:3)');
 rows(k,:)=[n.time,position,angle,n.positionErrorBound_m,n.attitudeErrorBound_rad,n.positionErrorBound_m-position,n.attitudeErrorBound_rad-angle];
end
valid=all(isfinite(rows),2);r=rows(valid,:);positionBad=r(:,6)<-1e-8;attitudeBad=r(:,7)<-1e-8;
result=struct('passed',~isempty(records)&&isempty(missing)&&isempty(malformed)&&~any(positionBad)&&~any(attitudeBad),...
 'records',numel(records),'matched_valid_records',size(r,1),'missing_truth_times_s',missing,'malformed_record_indices',malformed,...
 'position_violations',sum(positionBad),'attitude_violations',sum(attitudeBad),...
 'position_violation_times_s',r(positionBad,1)','attitude_violation_times_s',r(attitudeBad,1)',...
 'maximum_actual_position_error_m',[],'maximum_actual_attitude_error_rad',[],...
 'minimum_position_slack_m',[],'minimum_attitude_slack_rad',[],...
 'maximum_position_error_time_s',[],'maximum_attitude_error_time_s',[]);
if ~isempty(r)
 [result.maximum_actual_position_error_m,k]=max(r(:,2));result.maximum_position_error_time_s=r(k,1);
 [result.maximum_actual_attitude_error_rad,k]=max(r(:,3));result.maximum_attitude_error_time_s=r(k,1);
 result.minimum_position_slack_m=min(r(:,6));result.minimum_attitude_slack_rad=min(r(:,7));
end
end
