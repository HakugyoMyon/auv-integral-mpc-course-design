function trim=auv6_full_trim(U,p,f)
% Unknown phi,theta,vertical pair, horizontal common/differential, thrust.
lo=[deg2rad(-5);deg2rad(-20);deg2rad(-15);deg2rad(-12);deg2rad(-3);0];
hi=-lo;hi(6)=max(p.staticThrust);
[z,~,res,flag]=lsqnonlin(@residual,[0;-.06;0;.03;0;max(3,10*U^2)],lo,hi,...
 optimoptions('lsqnonlin','Display','off','FunctionTolerance',1e-14,'StepTolerance',1e-12,'OptimalityTolerance',1e-12,'MaxFunctionEvaluations',3000));
[x,u]=make(z);assert(flag>0 && norm(res)<1e-6,'Full trim failed');
trim=struct('speed_m_s',U,'state',x,'control',u,'residual_wrench',res,'solver_flag',flag);
 function [xx,uu]=make(z)
  y=zeros(14,1);y(1:2)=z(1:2);xx=auv6_from_euler_state(y);
  ph=z(1);th=z(2);R=[cos(th),sin(ph)*sin(th),cos(ph)*sin(th);0,cos(ph),-sin(ph);-sin(th),sin(ph)*cos(th),cos(ph)*cos(th)];
  xx(8:10)=R'*[U;0;0];xx(14)=z(6);
  xx(15:18)=[z(3);z(3);z(4)+z(5);z(4)-z(5)];uu=[z(6);xx(15:18)];
 end
 function rr=residual(z)
  [xx,~]=make(z);tau=auv6_fin_loads(xx(8:13),xx(15:18),f);
  dd=auv6_rhs(xx(1:14),1500,tau,p);rr=p.M*dd(8:13);
 end
end
