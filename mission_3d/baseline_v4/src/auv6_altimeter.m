function packet=auv6_altimeter(x,t,c,terrain,stream,forcedDropout)
% Only sensor simulator sees the terrain. Four central rays, no acoustic solver.
R=auv6_rotation(x(4:7));origin=x(1:3)+R*c.mount;dirs=R*c.beams;
ranges=nan(1,4);valid=false(1,4);
for k=1:4
 v=dirs(:,k);if v(3)<=0,continue;end
 % Bracket the first downward intersection in a smooth single-valued terrain.
 lo=0;hi=c.maxRange;
 if height(lo)>=0 || height(hi)<0,continue;end
 for step=1:25
  mid=(lo+hi)/2;if height(mid)>0,hi=mid;else,lo=mid;end
 end
 range=(lo+hi)/2;hit=origin+range*v;h=.01;
 dn=(terrain(hit(1)+h,hit(2))-terrain(hit(1)-h,hit(2)))/(2*h);
 de=(terrain(hit(1),hit(2)+h)-terrain(hit(1),hit(2)-h))/(2*h);
 normal=[-dn;-de;1];normal=normal/norm(normal);
 incidence=acos(max(-1,min(1,dot(v,normal))));
 valid(k)=range>=c.minRange && range<=c.maxRange && incidence<c.maxIncidence;
 if valid(k),ranges(k)=range+c.noiseStd*randn(stream);end
end
drop=forcedDropout || rand(stream)<c.dropoutProbability;
if drop,valid(:)=false;end
packet=struct('time',t,'availableTime',t+c.latency,'ranges',ranges,'beamValid',valid,...
 'valid',all(valid),'attitudeAtSample',x(4:7),'depthAtSample',x(3));
 function value=height(range)
  p=origin+range*v;value=p(3)-terrain(p(1),p(2));
 end
end
