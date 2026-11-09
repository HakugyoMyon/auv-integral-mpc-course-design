function [map,event]=sta_map_update(map,packet,nav)
% Nav must be the capture-time body pose; do not map delayed scans at NOW pose.
event=struct('accepted',false,'reason','invalid_packet_or_navigation','newGuardVoxels',0,'frameCount',numel(map.frames));
if ~packet.scanValid||~nav.valid,return;end
required={'positionNED','quaternion','positionErrorBound_m','attitudeErrorBound_rad','time'};
assert(all(isfield(nav,required)),'Explicit capture-time pose/error contract required.');
if packet.time<=map.lastPacketTime,event.reason='duplicate_or_out_of_order';return;end
if abs(nav.time-packet.time)>1e-6,event.reason='pose_not_at_capture_time';return;end
if any(~isfinite([nav.positionNED(:);nav.quaternion(:);nav.positionErrorBound_m;nav.attitudeErrorBound_rad]))||...
 nav.positionErrorBound_m<0||nav.attitudeErrorBound_rad<0||nav.attitudeErrorBound_rad>=pi/4,return;end
c=map.configuration;R=auv6_rotation(nav.quaternion);origin=nav.positionNED(:)+R*packet.mountFRD;
% Attitude uncertainty displaces the sonar origin as well as its rays.
originError=nav.positionErrorBound_m+2*norm(packet.mountFRD)*sin(nav.attitudeErrorBound_rad/2);
prefix=packet.range_m-packet.rangeErrorBound_m;
prefix(packet.rayState==0|~isfinite(prefix))=0;prefix=max(0,min(prefix,packet.maxRange_m));
fields=cell(size(c.minimumGridHalfWidths));
for k=1:numel(fields)
 w=c.minimumGridHalfWidths(k);fields{k}=movmin(movmin(prefix,[w,w],1,'Endpoints','shrink'),[w,w],2,'Endpoints','shrink');
end
frame=struct('originNED',origin,'rotation',R,'prefix_m',prefix,'minimum',{fields},...
 'azimuth_rad',packet.azimuth_rad,'elevation_rad',packet.elevation_rad,...
 'positionErrorBound_m',originError,'attitudeErrorBound_rad',nav.attitudeErrorBound_rad,...
 'minRange_m',packet.minRange_m,'time',packet.time);
hit=packet.rayState==1&isfinite(packet.range_m)&packet.range_m>=packet.minRange_m;
[az,el]=meshgrid(packet.azimuth_rad,packet.elevation_rad);dist=packet.range_m(hit);
local=[cos(el(hit)).*cos(az(hit)),cos(el(hit)).*sin(az(hit)),sin(el(hit))]';
points=(origin+R*(local.*dist'))';res=c.voxelSize_m;voxel=round(points/res);
errors=originError+2*dist*sin(nav.attitudeErrorBound_rad/2)+packet.rangeErrorBound_m+sqrt(3)*res/2;
allVoxel=[round(map.occupiedCentersNED/res);voxel];allErrors=[map.occupiedError_m;errors];
if isempty(allVoxel),uniqueVoxel=zeros(0,3);merged=zeros(0,1);
else,[uniqueVoxel,~,ids]=unique(allVoxel,'rows');merged=accumarray(ids,allErrors,[],@max);end
if size(uniqueVoxel,1)>c.maxOccupiedVoxels,map.capacityExceeded=true;event.reason='occupied_capacity_limit';return;end
oldCount=size(map.occupiedCentersNED,1);map.occupiedCentersNED=uniqueVoxel*res;map.occupiedError_m=merged;
map.frames{end+1}=frame;if numel(map.frames)>c.maxFrames,map.frames=map.frames(end-c.maxFrames+1:end);end
map.lastPacketTime=packet.time;map.acceptedScans=map.acceptedScans+1;
event.accepted=true;event.reason='captured_static_scan';event.newGuardVoxels=size(uniqueVoxel,1)-oldCount;event.frameCount=numel(map.frames);
end
