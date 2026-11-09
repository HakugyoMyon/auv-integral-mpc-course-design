function report=d11_audit_heading_innovations(file)
% Reconstruct nominal bias and actual NIS decisions from saved raw packets.
% Does not use true yaw for any filter update. Truth only labels outliers.
z=load(file,'h','c','prep');h=z.h;c=z.c;R=auv6_rotation(h.estimate(1,4:7)');
s=struct('p',h.estimate(1,1:3)'+R*c.rINS,...
 'v',R*(h.estimate(1,8:10)'+cross(h.estimate(1,11:13)',c.rINS)),...
 'q',h.estimate(1,4:7)','bg',z.prep.calibration.estimate,'ba',zeros(3,1),...
 'headingBias',0,'P',h.navigationCovariance(:,:,1));
s.bg(3)=z.prep.alignment.gyroBiasZ;
last=floor((h.t(end)-c.lag)/c.dt+1e-6)+1;
events=zeros(0,5);maxPacketYawDiscrepancy=0;
for k=1:last
 if k>1,s=d11_nav_predict(s,h.imu(:,k-1),c);end
 for j=1:numel(h.packets{k})
  packet=h.packets{k}{j};[s,info]=d11_nav_update(s,packet,c);
  if strcmp(packet.type,'heading')
   time=(k-1)*c.dt;row=round(time/.1)+1;
   assert(abs(h.t(row)-time)<1e-7);
   trueEuler=auv6_euler_state(h.truth(row,:)');
   discrepancy=atan2(sin(packet.z-trueEuler(3)),cos(packet.z-trueEuler(3)));
   events(end+1,:)=[time,info.nis,info.accepted,rad2deg(discrepancy),rad2deg(s.headingBias)];
   maxPacketYawDiscrepancy=max(maxPacketYawDiscrepancy,abs(discrepancy));
  end
 end
end
% Verify reconstruction against the recorded current-time estimate rather
% than trusting a plausible innovation log alone.
for k=last:size(h.imu,2),s=d11_nav_predict(s,h.imu(:,k),c);end
R=auv6_rotation(s.q);position=s.p-R*c.rINS;
reconstructionError=max([norm(position-h.estimate(end,1:3)'),norm(s.q-h.estimate(end,4:7)'),...
 norm(s.P-h.navigationCovariance(:,:,end),'fro')]);
assert(reconstructionError<1e-8,'Raw replay does not reproduce recorded navigation.');
report=struct('passed',true,'accepted_heading_packets',sum(events(:,3)==1),'rejected_heading_packets',sum(events(:,3)==0),...
 'accepted_packets_with_truth_discrepancy_over_10deg',sum(events(:,3)==1&abs(events(:,4))>10),...
 'maximum_packet_truth_discrepancy_deg',rad2deg(maxPacketYawDiscrepancy),...
 'final_reconstructed_common_bias_deg',rad2deg(s.headingBias),'navigation_reconstruction_error',reconstructionError,...
 'scope','Raw-packet chronological ESKF re-evaluation. True heading labels packet error only, never updates filter. No magnetic-disturbance immunity proof.');
save(fullfile(fileparts(file),'heading_innovations.mat'),'events','report');
fid=fopen(fullfile(fileparts(file),'heading_innovations.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
end
