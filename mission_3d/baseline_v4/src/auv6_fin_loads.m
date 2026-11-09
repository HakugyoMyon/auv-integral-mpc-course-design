function [tau,detail]=auv6_fin_loads(nuRelative,delta,f)
% nuRelative at body reference origin. Positive angle yields +normal force
% in forward flow. Mechanical servo sign mapping remains to be calibrated.
assert(numel(nuRelative)==6 && numel(delta)==4);
tau=zeros(6,1); forces=zeros(3,4); alpha=zeros(4,1);
for i=1:4
 r=f.position(:,i); n=f.normal(:,i); ex=[1;0;0];
 velocity=nuRelative(1:3)+cross(nuRelative(4:6),r);
 u=dot(velocity,ex); vn=dot(velocity,n); speed=hypot(u,vn);
 if speed<1e-8,continue;end
 d=min(max(delta(i),-f.angleLimit),f.angleLimit);
 alpha(i)=atan2(sin(d-atan2(vn,u)),cos(d-atan2(vn,u)));
 % No claim of valid reverse-flow lift or post-stall model.
 cl=0;
 if u>0
  cl=f.liftSlope*min(max(alpha(i),-f.alphaLimit),f.alphaLimit);
 end
 cd=f.CD0+cl^2/(pi*f.efficiency*f.AR);
 vhat=(u*ex+vn*n)/speed;
 liftDirection=(-vn*ex+u*n)/speed;
 force=.5*f.rho*speed^2*f.area*(cl*liftDirection-cd*vhat);
 forces(:,i)=force;tau=tau+[force;cross(r,force)];
end
detail=struct('force',forces,'alpha',alpha,'reverseFlowUnsupported',nuRelative(1)<=0);
end
