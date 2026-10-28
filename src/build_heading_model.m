function model = build_heading_model(trim,Ts,operatingZ,previousRudder,validate)
% Successive local AFFINE model; no current/disturbance truth is available.
if nargin<3, operatingZ=[trim.x;trim.a]; end
if nargin<4, previousRudder=operatingZ(13); end
if nargin<5, validate=true; end
indices=[2 4 6 10 12 13]; steps=1e-5*ones(6,1); actuatorParameters=auv_parameters(); tau=actuatorParameters.tau(1);
origin=operatingZ; [A,B]=differentiate(1); changes=zeros(7,1);
if validate
 [AA,BB]=differentiate(.5); [AAA,BBB]=differentiate(2);
 for j=1:7
  if j<=6, base=A(:,j); fine=AA(:,j); coarse=AAA(:,j);
  else, base=B; fine=BB; coarse=BBB; end
  absolute=max(norm(base-fine,inf),norm(base-coarse,inf));
  changes(j)=absolute/max(norm(base,inf),1e-8);
  assert(absolute<=max(1e-8,1e-3*norm(base,inf)),'AUV:Jacobian','Finite difference sensitivity too large.');
 end
end
f0=local_rhs(origin); exponential=expm([A B f0;zeros(2,8)]*Ts);
model=struct('Ac',A,'Bc',B,'affineContinuous',f0,'Ad',exponential(1:6,1:6), ...
 'Bd',exponential(1:6,7),'gd',exponential(1:6,8),'Ts',Ts, ...
 'indices',indices,'steps',steps,'relativeColumnChanges',changes, ...
 'origin',origin(indices),'inputOrigin',previousRudder,'trimRudder',trim.a(1), ...
 'stateNames',{{'v','p','r','phi','psi','delta_r'}});
 function f=local_rhs(zz)
  % Hull loads depend on ACTUAL rudder, not commanded rudder.
  dx=remus100(zz(1:12),zz(13:15),0,0,0);
  f=[dx(indices(1:5));(previousRudder-zz(13))/tau];
 end
 function [AA,BB]=differentiate(factor)
  AA=zeros(6); BB=[zeros(5,1);1/tau];
  for k=[1 2 3 4 6]
   zp=origin; zm=origin; delta=steps(k)*factor;
   zp(indices(k))=zp(indices(k))+delta; zm(indices(k))=zm(indices(k))-delta;
   AA(:,k)=(local_rhs(zp)-local_rhs(zm))/(2*delta);
  end
 end
end
