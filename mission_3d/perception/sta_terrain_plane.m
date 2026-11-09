function obs=sta_terrain_plane(packet,xhatCapture,c,previous)
% Received range packet + CAPTURE-time estimated body pose only. No world.
% Height+slope prediction motivation: doi:10.3390/jmse12030366 Sec.2.2.
% This helper does not implement that paper's sensor layout or controller.
if nargin<4,previous=[];end
obs=struct('valid',false,'time',packet.time,'captureTime',packet.time,...
 'originNE',xhatCapture(1:2),'bottomDepth_m',NaN,'rawGradientNE',[NaN;NaN],...
 'gradientNE',[NaN;NaN],'residual_m',NaN,'gradientClipped',false,...
 'source','received_four_beam_altimeter_capture_pose',...
 'scope','Local affine bottom prediction from received four-beam ranges and capture-time estimated pose. Smoothed/clipped slope is nominal planning information, not a true terrain bound or clearance guarantee. Invalid observations must not refresh last-valid age; last valid is unusable after 0.7s.');
baseline=auv6_altitude_estimate(packet,xhatCapture(4:7),c);obs.residual_m=baseline.planeResidual;
if ~baseline.valid,return;end
R=auv6_rotation(xhatCapture(4:7));points=R*(c.mount+c.beams.*packet.ranges);
design=[points(1:2,:)',ones(4,1)];plane=design\points(3,:)';
raw=plane(1:2);filtered=raw;
if ~isempty(previous)&&isfield(previous,'valid')&&previous.valid&&isfield(previous,'gradientNE')&&...
  all(isfinite(previous.gradientNE))&&isfield(previous,'time')&&packet.time>previous.time
 dt=packet.time-previous.time;weight=dt/(2+dt);filtered=previous.gradientNE(:)+weight*(raw-previous.gradientNE(:));
end
limit=.07;if norm(filtered)>limit,filtered=limit*filtered/norm(filtered);obs.gradientClipped=true;end
obs.valid=true;obs.bottomDepth_m=xhatCapture(3)+plane(3);obs.rawGradientNE=raw;obs.gradientNE=filtered;
end
