function obs=auv6_altitude_estimate(packet,qEstimate,c)
% No terrain or true clearance input. Fit observed bottom plane at body origin.
obs=struct('valid',false,'altitude',NaN,'planeResidual',NaN);
if ~packet.valid || ~all(isfinite(packet.ranges)),return;end
R=auv6_rotation(qEstimate);if R(3,3)<cos(c.maxTilt),return;end
points=R*(c.mount+c.beams.*packet.ranges);
design=[points(1:2,:)',ones(4,1)];
if rcond(design'*design)<1e-8,return;end
plane=design\points(3,:)';res=norm(design*plane-points(3,:)')/2;
obs.altitude=plane(3);obs.planeResidual=res;
obs.valid=res<=c.maxPlaneResidual && obs.altitude>0;
end
