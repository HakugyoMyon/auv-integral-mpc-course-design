function [dx,alpha,deltaRate]=d8_preview_rhs(x,targetThrust,deltaCommand,p,f,cache)
% Algebraic batch evaluation of frozen auv6_actuated_rhs for still water.
% No dynamics removed, timestep enlarged, or hydrodynamic limits changed.
r=cache.position;n=cache.normal;nu=x(8:13);v=nu(1:3);w=nu(4:6);
velocity=[v(1)+w(2)*r(3,:)-w(3)*r(2,:);...
 v(2)+w(3)*r(1,:)-w(1)*r(3,:);v(3)+w(1)*r(2,:)-w(2)*r(1,:)];
u=velocity(1,:);vn=sum(velocity.*n,1);speed=hypot(u,vn);active=speed>=1e-8;
denominator=speed;denominator(~active)=1;
d=min(max(x(15:18)',-f.angleLimit),f.angleLimit);
angle=d-atan2(vn,u);alpha=atan2(sin(angle),cos(angle));alpha(~active)=0;
cl=f.liftSlope*min(max(alpha,-f.alphaLimit),f.alphaLimit);cl(u<=0)=0;
cd=f.CD0+cl.^2/cache.dragFactor;
ex=[ones(1,4);zeros(2,4)];
force=.5*f.rho*f.area*speed.^2.*(cl.*(-vn.*ex+u.*n)./denominator-cd.*(u.*ex+vn.*n)./denominator);
force(:,~active)=0;
moment=[r(2,:).*force(3,:)-r(3,:).*force(2,:);...
 r(3,:).*force(1,:)-r(1,:).*force(3,:);r(1,:).*force(2,:)-r(2,:).*force(1,:)];
finWrench=[sum(force,2);sum(moment,2)];
q=x(4:7);unit=q/norm(q);s=unit(1);e=unit(2:4);
S=[0,-e(3),e(2);e(3),0,-e(1);-e(2),e(1),0];R=eye(3)+2*s*S+2*S*S;
momentum=p.M*nu;
convective=[cross3(w,momentum(1:3));cross3(v,momentum(1:3))+cross3(w,momentum(4:6))];
drag=(p.d1+p.d2.*abs(nu)).*nu;
down=R'*[0;0;1];gravity=p.W*down;buoyancy=-p.B*down;
hydrostatic=[gravity+buoyancy;cross3(p.rG,gravity)+cross3(p.rB,buoyancy)];
thrust=[x(14);0;0];tauT=[thrust;cross3(p.thrusterPosition,thrust)];
nuDot=cache.inverseMass*(hydrostatic+tauT+finWrench-convective-drag);
qDot=.5*[-q(2:4)'*w;q(1)*w+cross3(q(2:4),w)];
target=min(max(deltaCommand,-f.angleLimit),f.angleLimit);
deltaRate=min(max((target-x(15:18))/f.lag,-f.rateLimit),f.rateLimit);
dx=[R*v;qDot;nuDot;(targetThrust-x(14))/p.thrusterLag_s;deltaRate];alpha=alpha';
end
function c=cross3(a,b)
c=[a(2)*b(3)-a(3)*b(2);a(3)*b(1)-a(1)*b(3);a(1)*b(2)-a(2)*b(1)];
end
