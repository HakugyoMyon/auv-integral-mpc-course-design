function quality=auv12b_nav_quality(s,c,time,lastDVL)
R=auv6_rotation(s.q);H=zeros(3,15);H(:,1:3)=eye(3);H(:,7:9)=R*auv6_skew(c.rINS);
P=H*s.P*H';PNE=(P(1:2,1:2)+P(1:2,1:2)')/2;
% psi gradient of a right rotation, evaluated by finite differences.
g=zeros(1,3);eps=1e-6;
for k=1:3
 d=zeros(3,1);d(k)=eps;qp=auv6_qmul(s.q,[cos(eps/2);sin(eps/2)*d/eps]);qm=auv6_qmul(s.q,[cos(eps/2);-sin(eps/2)*d/eps]);
 yp=auv6_euler_state([zeros(3,1);qp;zeros(11,1)]);ym=auv6_euler_state([zeros(3,1);qm;zeros(11,1)]);
 g(k)=atan2(sin(yp(3)-ym(3)),cos(yp(3)-ym(3)))/(2*eps);
end
sigmaYaw=sqrt(max(0,g*s.P(7:9,7:9)*g'));radius=sqrt(-2*log(.05)*max(eig(PNE)));
quality=struct('PNE',PNE,'radius95',radius,'yawStd',sigmaYaw,'dvlAge',time-lastDVL,...
 'valid',all(isfinite(PNE),'all') && min(eig(PNE))>=0 && time-lastDVL<=1 && sigmaYaw<=deg2rad(.5));
end
