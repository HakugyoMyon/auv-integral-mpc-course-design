function [s,info]=d11_nav_update(s,m,c)
% All measurements jointly update full covariance, including common bias.
R=auv6_rotation(s.q);H=zeros(numel(m.z),16);
switch m.type
 case 'dvl'
  r=c.rDVL-c.rINS;vb=R'*s.v;pred=vb+cross(m.gyro-s.bg,r);
  H(:,4:6)=R';H(:,7:9)=auv6_skew(vb);H(:,10:12)=auv6_skew(r);
 case 'pressure'
  r=c.rPressure-c.rINS;pred=s.p(3)+R(3,:)*r;H(1,3)=1;H(1,7:9)=-R(3,:)*auv6_skew(r);
 case 'gnss'
  r=c.rGNSS-c.rINS;pred=s.p+R*r;H(:,1:3)=eye(3);H(:,7:9)=-R*auv6_skew(r);
 case 'heading'
  e=auv6_euler_state([zeros(3,1);s.q;zeros(11,1)]);
  if ~isfield(m,'valid')||~m.valid||max(abs(e(1:2)))>c.headingAid.maximumTilt_rad
   info=struct('nis',NaN,'accepted',false,'reason','invalid_or_tilt');return;
  end
  pred=e(3)+s.headingBias;epsilon=1e-6;
  for k=1:3
   d=zeros(3,1);d(k)=sin(epsilon/2);
   qp=auv6_qmul(s.q,[cos(epsilon/2);d]);qm=auv6_qmul(s.q,[cos(epsilon/2);-d]);
   ep=auv6_euler_state([zeros(3,1);qp;zeros(11,1)]);em=auv6_euler_state([zeros(3,1);qm;zeros(11,1)]);
   H(1,k+6)=atan2(sin(ep(3)-em(3)),cos(ep(3)-em(3)))/(2*epsilon);
  end
  H(1,16)=1;
 otherwise,error('Unknown measurement type.');
end
innovation=m.z-pred;if strcmp(m.type,'heading'),innovation=atan2(sin(innovation),cos(innovation));end
S=H*s.P*H'+m.covariance;nis=innovation'*(S\innovation);gate=16.3;
if numel(m.z)==1,gate=10.83;end
info=struct('nis',nis,'accepted',isfinite(nis)&&nis<gate,'predicted',pred,'H',H);
if ~info.accepted,return;end
K=(s.P*H')/S;dx=K*innovation;J=eye(16)-K*H;s.P=J*s.P*J'+K*m.covariance*K';
s.p=s.p+dx(1:3);s.v=s.v+dx(4:6);s.q=auv6_qmul(s.q,[1;.5*dx(7:9)]);
s.bg=s.bg+dx(10:12);s.ba=s.ba+dx(13:15);s.headingBias=s.headingBias+dx(16);
reset=eye(16);reset(7:9,7:9)=eye(3)-.5*auv6_skew(dx(7:9));
s.P=reset*s.P*reset';s.P=(s.P+s.P')/2;
end
