function packet=d11_payload_sensor(x,time,c,terrain,stream,forcedDropout)
% ONLY simulator sees terrain/truth. Abstract continuous imaging footprint;
% no acoustic propagation, defect recognition or measured instrument claim.
R=auv6_rotation(x(4:7));origin=x(1:3)+R*c.mount;directions=R*c.rays;
ranges=nan(1,5);hits=nan(3,5);
for k=1:5
 direction=directions(:,k);lo=0;hi=c.maximumRange_m;
 if direction(3)<=0||residual(lo)>=0||residual(hi)<0,continue;end
 for j=1:25
  mid=(lo+hi)/2;if residual(mid)>=0,hi=mid;else,lo=mid;end
 end
 ranges(k)=(lo+hi)/2;hits(:,k)=origin+ranges(k)*direction;
end
valid=all(isfinite(ranges))&&ranges(5)>=c.minimumAltitude_m;
if valid
 % Declared smooth, locally planar seabed domain. Reject curved patches
 % rather than claiming an unobserved concavity is continuously imaged.
 A=[hits(1:2,:)',ones(5,1)];plane=A\hits(3,:)';maxDeviation=0;
 for u=[.25,.5,.75]
  for v=[.25,.5,.75]
   p=(1-u)*(1-v)*hits(1:2,1)+u*(1-v)*hits(1:2,2)+u*v*hits(1:2,3)+(1-u)*v*hits(1:2,4);
   maxDeviation=max(maxDeviation,abs(terrain(p(1),p(2))-[p',1]*plane));
  end
 end
 valid=maxDeviation<=c.maximumPlaneDeviation_m;
end
e=auv6_euler_state(x);valid=valid&&max(abs(e(1:2)))<=c.maximumTilt_rad;
valid=valid&&~forcedDropout&&rand(stream)>=c.dropoutProbability;
if all(isfinite(ranges)),ranges=ranges+c.rangeStd_m*randn(stream,1,5);end
packet=struct('schema','auv.geometric_payload.v1','time',time,'availableTime',time+c.latency_s,...
 'valid',valid,'ranges_m',ranges,'model','continuous_geometric_imaging_footprint_on_locally_planar_bottom');
 function value=residual(distance)
  point=origin+distance*direction;value=point(3)-terrain(point(1),point(2));
 end
end
