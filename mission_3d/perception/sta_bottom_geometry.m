function [origin,hits,ranges]=sta_bottom_geometry(x,mount,rays,maxRange,terrain)
% Sensor-private noiseless terrain endpoints, same bisection as baseline.
R=auv6_rotation(x(4:7));origin=x(1:3)+R*mount;directions=R*rays;
ranges=nan(1,size(rays,2));hits=nan(3,size(rays,2));
for k=1:size(rays,2)
 v=directions(:,k);lo=0;hi=maxRange;
 if v(3)<=0||residual(lo)>=0||residual(hi)<0,continue;end
 for j=1:25,mid=(lo+hi)/2;if residual(mid)>=0,hi=mid;else,lo=mid;end;end
 ranges(k)=(lo+hi)/2;hits(:,k)=origin+ranges(k)*v;
end
 function r=residual(d),p=origin+d*v;r=p(3)-terrain(p(1),p(2));end
end
