function [valid,peakAlpha,predictedState]=d15_runtime_preview(x,u,p,f)
% Every MPC tick checks its actual newly computed command over one control
% interval, using the same 100Hz nonlinear plant integrator.
cmd=[auv6_thrust_pwm(u(1),p);u(2:5)];peakAlpha=0;
for k=1:10
 [a,aa]=auv6_actuated_rhs(x,cmd,p,f);[b,ab]=auv6_actuated_rhs(x+.005*a,cmd,p,f);
 [c,ac]=auv6_actuated_rhs(x+.005*b,cmd,p,f);[d,ad]=auv6_actuated_rhs(x+.01*c,cmd,p,f);
 peakAlpha=max([peakAlpha;abs(aa.fin.alpha);abs(ab.fin.alpha);abs(ac.fin.alpha);abs(ad.fin.alpha)]);
 x=x+.01/6*(a+2*b+2*c+d);x(4:7)=x(4:7)/norm(x(4:7));
end
valid=peakAlpha<=deg2rad(14.5) && max(abs(x(15:18)))<=deg2rad(8.05);
predictedState=x;
end
