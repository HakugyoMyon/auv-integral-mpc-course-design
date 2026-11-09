function observation=d11_payload_observe(packet,sampleNavigation,sampleQuality,attitudeCovariance,centersNE,cellRadius,c,now)
% No terrain or physical truth. Navigation must correspond to packet sample.
observation=struct('valid',false,'indices',[],'polygonNE',zeros(0,2),'erosion_m',Inf,'time',packet.time);
if ~packet.valid||~sampleQuality.valid||now<packet.availableTime||now-packet.time>.5,return;end
if numel(packet.ranges_m)~=5||any(~isfinite(packet.ranges_m))||any(packet.ranges_m<=0)||any(packet.ranges_m>c.maximumRange_m+.1),return;end
if ~all(isfinite(attitudeCovariance),'all')||min(eig((attitudeCovariance+attitudeCovariance')/2))<0,return;end
R=auv6_rotation(sampleNavigation(4:7));points=sampleNavigation(1:3)+R*(c.mount+c.rays.*packet.ranges_m);
polygon=points(1:2,1:4)';hull=convhull(polygon(:,1),polygon(:,2));polygon=polygon(hull(1:end-1),:);
orientationBound=3*sqrt(max(eig(attitudeCovariance)));
erosion=sampleQuality.radius95+cellRadius+c.geometryMargin_m+3*c.rangeStd_m+max(packet.ranges_m)*orientationBound;
inside=inpolygon(centersNE(:,1),centersNE(:,2),polygon(:,1),polygon(:,2));
distance=inf(size(centersNE,1),1);
for k=1:size(polygon,1)
 a=polygon(k,:);b=polygon(mod(k,size(polygon,1))+1,:);edge=b-a;
 if norm(edge)<1e-8,return;end
 distance=min(distance,abs(edge(1)*(centersNE(:,2)-a(2))-edge(2)*(centersNE(:,1)-a(1)))/norm(edge));
end
observation.valid=true;observation.indices=find(inside&distance>=erosion);
observation.polygonNE=polygon;observation.erosion_m=erosion;
end
