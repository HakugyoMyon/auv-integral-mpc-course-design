function report=d11_audit_heading_run(file)
s=load(file,'h','plan','mission','report','c','memoryLog');h=s.h;c=s.c;
assert(size(h.navigationCovariance,1)==16,'Missing persistent heading-bias covariance.');
assert(isequal(h.appliedCommandDuration_s,[diff(h.t);0]));
minimumEigenvalue=Inf;maximumYawStd=0;maxPositionError=0;maxYawError=0;
for k=1:numel(h.t)
 P=h.navigationCovariance(:,:,k);assert(all(isfinite(P),'all'));
 minimumEigenvalue=min(minimumEigenvalue,min(eig((P+P')/2)));
 state=struct('P',P(1:15,1:15),'q',h.estimate(k,4:7)');
 q=auv12b_nav_quality(state,c,h.t(k),h.t(k)-h.quality(k,3));
 assert(abs(q.yawStd-h.quality(k,2))<1e-10&&abs(q.radius95-h.quality(k,1))<1e-9);
 maximumYawStd=max(maximumYawStd,q.yawStd);
 maxPositionError=max(maxPositionError,norm(h.truth(k,1:2)-h.estimate(k,1:2)));
 et=auv6_euler_state(h.truth(k,:)');ee=auv6_euler_state(h.estimate(k,:)');
 maxYawError=max(maxYawError,abs(atan2(sin(et(3)-ee(3)),cos(et(3)-ee(3)))));
end
headingCount=0;gnssCount=0;
for k=1:numel(h.packets)
 for j=1:numel(h.packets{k})
  packet=h.packets{k}{j};gnssCount=gnssCount+strcmp(packet.type,'gnss');
  if strcmp(packet.type,'heading')
   headingCount=headingCount+1;assert(isfinite(packet.z)&&islogical(packet.valid));
   assert(abs(packet.covariance-c.headingAid.whiteStd_rad^2)<1e-16);
  end
 end
end
assert(gnssCount==0,'Underwater GNSS is forbidden.');
assert(headingCount>0&&minimumEigenvalue>=-1e-10);
assert(min(s.memoryLog(:,2))>=.75,'Resource floor violated.');
report=struct('passed',true,'heading_packets',headingCount,'underwater_GNSS_packets',gnssCount,...
 'minimum_covariance_eigenvalue',minimumEigenvalue,'maximum_reported_yawStd_deg',rad2deg(maximumYawStd),...
 'maximum_observed_yaw_error_deg',rad2deg(maxYawError),'maximum_observed_horizontal_error_m',maxPositionError,...
 'minimum_available_RAM_GB',min(s.memoryLog(:,2)),'physical_task_pass',s.report.physical_and_task_pass,...
 'scope','Recorded covariance/quality and packet audit. Actual errors are observed samples, not proof of calibrated confidence, magnetic immunity or operational safety.');
fid=fopen(fullfile(fileparts(file),'heading_audit.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
end
