function trim = compute_trim()
% Seven equations: six BODY accelerations and zero NED depth rate.
lb = [-.3;-.3;-deg2rad(10);-deg2rad(10);-deg2rad(5);-deg2rad(5);200];
ub = -lb; ub(7)=1525;
scale = [.1;.1;.1;.1;.1;.1;.1];
options = optimoptions('lsqnonlin','Display','off', ...
 'FunctionTolerance',1e-14,'StepTolerance',1e-14, ...
 'OptimalityTolerance',1e-12,'MaxFunctionEvaluations',5000, ...
 'TypicalX',[.01;.01;.01;.01;.01;.01;1000]);
[q,resnorm,residual,exitflag,output] = lsqnonlin(@equations, ...
 [0;0;.01;0;0;0;1000],lb,ub,options);
[x,a] = unpack(q);
dx=remus100(x,a,0,0,0);
trim=struct('x',x,'a',a,'unknown',q,'residualSI',dx([1:6 9]), ...
 'scaledResidual',residual,'resnorm',resnorm,'exitflag',exitflag, ...
 'solverOutput',output,'lower',lb,'upper',ub);
assert(exitflag>0 && max(abs(trim.residualSI))<1e-6,'AUV:Trim','Trim failed.');
 function e=equations(v)
  [xx,aa]=unpack(v); f=remus100(xx,aa,0,0,0); e=f([1:6 9])./scale;
 end
end
function [x,a]=unpack(q)
x=[1.5;q(1:2);0;0;0;0;0;10;q(3:4);0]; a=q(5:7);
end
