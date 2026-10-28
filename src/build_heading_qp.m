function qp = build_heading_qp(model,M,headingReference,previousRudder,P)
% Delta-scaled decision d = [Delta cmd(0)..Delta cmd(Nc-1)]/incrementScale.
assert(M.Nc<=M.Np && M.Np*model.Ts<=1.5+1e-12);
A=[model.Ad model.Bd;zeros(1,6) 1]; B=[model.Bd;1]; g=[model.gd;0];
F=zeros(7*M.Np,7); G=zeros(7*M.Np,M.Nc); affine=zeros(7*M.Np,1);
Fk=eye(7); Gk=zeros(7,M.Nc); ck=zeros(7,1);
for j=1:M.Np
 Fk=A*Fk; Gk=A*Gk; ck=A*ck+g;
 if j<=M.Nc, Gk(:,j)=Gk(:,j)+B; end
 rows=7*(j-1)+(1:7); F(rows,:)=Fk; G(rows,:)=Gk; affine(rows)=ck;
end
% Online linearization origin is the current measured state, including yaw.
z=[zeros(6,1);previousRudder-model.inputOrigin];
reference=zeros(7,1); reference(5)=angle(exp(1i*(headingReference-model.origin(5))));
% Physical cost is on r (not r-r_current) and command-trim (not command-previous).
reference(3)=-model.origin(3);
reference(7)=model.trimRudder-model.inputOrigin;
ref=repmat(reference,M.Np,1);
q=zeros(7,1); q(5)=M.qpsi/M.psiScale^2; q(3)=M.qr/M.rateScale^2; q(7)=M.qu/M.inputScale^2;
Q=spdiags(repmat(q,M.Np,1),0,7*M.Np,7*M.Np);
Gscaled=G*M.incrementScale; free=F*z+affine;
H=2*(Gscaled'*Q*Gscaled+M.rdelta*eye(M.Nc)); H=(H+H')/2;
f=2*Gscaled'*Q*(free-ref);
L=tril(ones(M.Nc)); Aineq=[L;-L]*M.incrementScale;
bineq=[repmat(P.upper(1)-previousRudder,M.Nc,1);repmat(previousRudder-P.lower(1),M.Nc,1)];
deltaMax=P.rate(1)*model.Ts/M.incrementScale;
qp=struct('H',H,'f',f,'A',Aineq,'b',bineq,'lb',-deltaMax*ones(M.Nc,1), ...
 'ub',deltaMax*ones(M.Nc,1),'F',F,'G',G,'affine',affine,'free',free,'Q',Q, ...
 'reference',ref,'incrementScale',M.incrementScale);
end
