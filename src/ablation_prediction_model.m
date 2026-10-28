function model = ablation_prediction_model(trim,Ts,z,previous,dimension,validate)
if nargin<6, validate=false; end
if dimension==6
 model=build_heading_model(trim,Ts,z,previous,validate);
 model.psiIndex=5; model.rateIndex=3; model.rudderIndex=6;
 return;
end
assert(dimension==10); indices=[1:6 10:13]; psiIndex=9;
P=auv_parameters(); tau=P.tau(1); steps=1e-5*ones(10,1);
[A,B]=differentiate(1); changes=zeros(11,1);
if validate
 [halfA,halfB]=differentiate(.5); [doubleA,doubleB]=differentiate(2);
 for j=1:11
  if j<=10, base=A(:,j); fine=halfA(:,j); coarse=doubleA(:,j);
  else, base=B; fine=halfB; coarse=doubleB; end
  absolute=max(norm(base-fine,inf),norm(base-coarse,inf));
  changes(j)=absolute/max(norm(base,inf),1e-8);
  assert(absolute<=max(1e-8,1e-3*norm(base,inf)),'AUV:Jacobian10','Ten-state Jacobian sensitivity failed.');
 end
end
f0=rhs(z); exponential=expm([A B f0;zeros(2,12)]*Ts);
model=struct('Ac',A,'Bc',B,'affineContinuous',f0,'Ad',exponential(1:10,1:10), ...
 'Bd',exponential(1:10,11),'gd',exponential(1:10,12),'Ts',Ts, ...
 'indices',indices,'steps',steps,'relativeColumnChanges',changes,'origin',z(indices), ...
 'inputOrigin',previous,'trimRudder',trim.a(1),'psiIndex',9,'rateIndex',6,'rudderIndex',10);
 function f=rhs(zz)
  dx=remus100(zz(1:12),zz(13:15),0,0,0);
  f=[dx(indices(1:9));(previous-zz(13))/tau];
 end
 function [AA,BB]=differentiate(factor)
  AA=zeros(10); BB=[zeros(9,1);1/tau];
  for k=setdiff(1:10,psiIndex)
   zp=z; zm=z; delta=steps(k)*factor;
   zp(indices(k))=zp(indices(k))+delta; zm(indices(k))=zm(indices(k))-delta;
   AA(:,k)=(rhs(zp)-rhs(zm))/(2*delta);
  end
 end
end
