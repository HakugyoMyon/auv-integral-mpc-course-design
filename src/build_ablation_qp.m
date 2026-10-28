function qp = build_ablation_qp(model,M,headingReference,previous,P,referenceRate,eta)
% Frozen base weights; only preview, integral state and prediction dimension vary.
Abl=M.ablation; ns=numel(model.origin); psi=model.psiIndex; rate=model.rateIndex;
if ~Abl.preview, referenceRate=0; end
if ns==6 && ~Abl.preview && ~Abl.integral
 qp=build_heading_qp(model,M,headingReference,previous,P);
 return;
end
assert(M.Nc<=M.Np && M.Np*model.Ts<=1.5+1e-12);
base=angle(exp(1i*(headingReference-model.origin(psi))));
nv=ns+1+double(Abl.integral); input=ns+1;
A=[model.Ad model.Bd;zeros(1,ns) 1]; B=[model.Bd;1]; g=[model.gd;0];
if Abl.integral
 A=[A zeros(ns+1,1);zeros(1,ns+1) 1]; A(end,psi)=-model.Ts;
 B=[B;0]; g=[g;0];
end
F=zeros(nv*M.Np,nv); G=zeros(nv*M.Np,M.Nc); affine=zeros(nv*M.Np,1);
reference=zeros(nv*M.Np,1); Fk=eye(nv); Gk=zeros(nv,M.Nc); ck=zeros(nv,1);
for j=1:M.Np
 gg=g;
 if Abl.integral, gg(end)=model.Ts*(base+referenceRate*(j-1)*model.Ts); end
 Fk=A*Fk; Gk=A*Gk; ck=A*ck+gg;
 if j<=M.Nc, Gk(:,j)=Gk(:,j)+B; end
 rows=nv*(j-1)+(1:nv); F(rows,:)=Fk; G(rows,:)=Gk; affine(rows)=ck;
 target=zeros(nv,1); target(psi)=base+referenceRate*j*model.Ts;
 target(rate)=-model.origin(rate); target(input)=model.trimRudder-model.inputOrigin;
 reference(rows)=target;
end
initial=[zeros(ns,1);previous-model.inputOrigin];
if Abl.integral, initial=[initial;eta]; end
q=zeros(nv,1); q(psi)=M.qpsi/M.psiScale^2; q(rate)=M.qr/M.rateScale^2; q(input)=M.qu/M.inputScale^2;
if Abl.integral, q(end)=Abl.qIntegral/Abl.integralScale^2; end
Q=spdiags(repmat(q,M.Np,1),0,nv*M.Np,nv*M.Np);
free=F*initial+affine; Gs=G*M.incrementScale;
H=2*(Gs'*Q*Gs+M.rdelta*eye(M.Nc)); H=(H+H')/2;
f=2*Gs'*Q*(free-reference);
L=tril(ones(M.Nc)); deltaMax=P.rate(1)*model.Ts/M.incrementScale;
qp=struct('H',H,'f',f,'A',[L;-L]*M.incrementScale, ...
 'b',[repmat(P.upper(1)-previous,M.Nc,1);repmat(previous-P.lower(1),M.Nc,1)], ...
 'lb',-deltaMax*ones(M.Nc,1),'ub',deltaMax*ones(M.Nc,1), ...
 'F',F,'G',G,'affine',affine,'free',free,'Q',Q,'reference',reference,'incrementScale',M.incrementScale);
end
