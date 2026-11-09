function m=d14w_mpc_build(p,f,operatingSpeed)
if nargin<3,operatingSpeed=.8;end
assert(operatingSpeed==.8,'Only the declared 0.8m/s branch is under validation.');
% Constrained multivariable MPC; no PID/LQR actuation policy or outer loop.
m.Ts=.1;m.blocks=[1 1 1 1 2 2 4 4 8 8 12 16];m.Np=sum(m.blocks);m.Nc=numel(m.blocks);
m.trim=auv6_full_trim(operatingSpeed,p,f);m.z0=auv_mpc_state(m.trim.state);m.z0(1)=0;m.u0=m.trim.control;
m.scale=[20;deg2rad([20 20 20 20])'];S=diag(m.scale);h=1e-5;A=zeros(15);B=zeros(15,5);
for k=1:15
 a=m.z0;b=m.z0;a(k)=a(k)+h;b(k)=b(k)-h;
 A(:,k)=(auv_mpc_dynamics(a,m.u0,p,f)-auv_mpc_dynamics(b,m.u0,p,f))/(2*h);
end
for k=1:5
 a=m.u0;b=m.u0;a(k)=a(k)+h;b(k)=b(k)-h;
 B(:,k)=(auv_mpc_dynamics(m.z0,a,p,f)-auv_mpc_dynamics(m.z0,b,p,f))/(2*h);
end
E=zeros(15,6);E(5:10,:)=eye(6);
sys=c2d(ss(A,[B*S,E],eye(15),zeros(15,11)),m.Ts);
m.Ad=sys.A;m.Bd=sys.B(:,1:5);m.Ed=sys.B(:,6:11);
weights=[16,1/deg2rad(5)^2,.1/deg2rad(10)^2,1/deg2rad(5)^2,...
 100,25,8,25,25,25,.0025,5,5,5,5];Q=diag(weights);
R=S'*diag([.0025;ones(4,1)*.8])*S;Rdu=S'*diag([.03;ones(4,1)*100])*S;
% Riccati terminal penalty only; no associated feedback gain is executed.
P=dare(m.Ad,m.Bd,Q,R);
nx=15;nu=5;N=m.Np;M=m.Nc;
m.F=zeros(N*nx,nx);m.G=zeros(N*nx,M*nu);m.J=zeros(N*nx,6);m.T=zeros(N*nu,M*nu);
blockIndex=repelem(1:M,m.blocks);stateMap=eye(nx);inputMap=zeros(nx,M*nu);distMap=zeros(nx,6);
for k=1:N
 selector=zeros(nu,M*nu);selector(:,(blockIndex(k)-1)*nu+(1:nu))=eye(nu);
 stateMap=m.Ad*stateMap;inputMap=m.Ad*inputMap+m.Bd*selector;distMap=m.Ad*distMap+m.Ed;
 rows=(k-1)*nx+(1:nx);m.F(rows,:)=stateMap;m.G(rows,:)=inputMap;m.J(rows,:)=distMap;
 m.T((k-1)*nu+(1:nu),:)=selector;
end
Qbar=kron(speye(N),Q);Qbar(end-nx+1:end,end-nx+1:end)=P;
Rbar=kron(speye(N),R);m.D=kron(eye(M)-diag(ones(M-1,1),-1),eye(nu));
Rdubar=kron(eye(M),Rdu);
m.GQ=m.G'*Qbar;m.TR=m.T'*Rbar;m.DR=m.D'*Rdubar;
H=2*(m.GQ*m.G+m.TR*m.T+m.DR*m.D);H=(H+H')/2;
m.H=blkdiag(H,2e6);assert(all(eig(m.H)>0));
selector=zeros(2,nx);selector(1,2)=1;selector(2,3)=1;
m.C=kron(speye(N),selector);Cg=m.C*m.G;
% Actual fin rate is largest just after each held-input block changes.
% Between changes the first-order actuator rate decays monotonically.
finSelector=zeros(4,nx);finSelector(:,12:15)=eye(4);
m.rateF=zeros(4*M,nx);m.rateJ=zeros(4*M,6);m.rateG=zeros(4*M,5*M);
starts=[1,1+cumsum(m.blocks(1:end-1))];
for k=1:M
 if starts(k)==1,Fpre=eye(nx);Gpre=zeros(nx,5*M);Jpre=zeros(nx,6);
 else,rr=(starts(k)-2)*nx+(1:nx);Fpre=m.F(rr,:);Gpre=m.G(rr,:);Jpre=m.J(rr,:);end
 rr=(k-1)*4+(1:4);target=zeros(4,5*M);target(:,(k-1)*5+(2:5))=diag(m.scale(2:5));
 m.rateF(rr,:)=-finSelector*Fpre/f.lag;m.rateJ(rr,:)=-finSelector*Jpre/f.lag;
 m.rateG(rr,:)=(target-finSelector*Gpre)/f.lag;
end
m.Aineq=[m.D,zeros(M*nu,1);-m.D,zeros(M*nu,1);Cg,-ones(2*N,1);-Cg,-ones(2*N,1);...
 m.rateG,zeros(4*M,1);-m.rateG,zeros(4*M,1)];
m.attitudeLimit=repmat(deg2rad([15;20]),N,1);
m.uMin=[0;-f.angleLimit*ones(4,1)];m.uMax=[max(p.staticThrust);f.angleLimit*ones(4,1)];
m.rate=[30;deg2rad(30)*ones(4,1)];m.observerGain=.03;m.disturbanceLimit=[.5*ones(3,1);ones(3,1)];
m.actualFinRate=f.rateLimit;
m.finLag=f.lag;
m.options=optimoptions('quadprog','Algorithm','interior-point-convex','Display','off','MaxIterations',100,'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-7);
m.stateOrder={'depth','roll','pitch','yaw','u','v','w','p','q','r','thrust','upper','lower','port','starboard'};
m.hessianCondition=cond(m.H);
m.Q=Q;m.R=R;m.Rdelta=Rdu;m.terminalPenalty=P;

% Hard linearized raw-fin attack constraints; nonlinear 6s gate is retained.
% 13deg is an explicit design reserve below the unchanged 15deg model limit.
z=m.z0;[~,detail]=auv6_fin_loads(z(5:10),z(12:15),f);alpha0=detail.alpha;
Calpha=zeros(4,15);
for k=1:15
 a=z;b=z;a(k)=a(k)+h;b(k)=b(k)-h;
 [~,da]=auv6_fin_loads(a(5:10),a(12:15),f);[~,db]=auv6_fin_loads(b(5:10),b(12:15),f);
 difference=atan2(sin(da.alpha-db.alpha),cos(da.alpha-db.alpha));Calpha(:,k)=difference/(2*h);
end
m.alphaSelector=kron(speye(m.Np),Calpha);m.alphaOrigin=repmat(alpha0,m.Np,1);
m.alphaLinearLimit=deg2rad(13);m.operatingSpeed=operatingSpeed;
alphaG=m.alphaSelector*m.G;
m.Aineq=[m.Aineq;alphaG,zeros(4*m.Np,1);-alphaG,zeros(4*m.Np,1)];
m.modelScope='Independent 0.8m/s linearization with hard predicted raw-attack bounds. Full nonlinear envelope remains separately checked.';
m.warmOptions=optimoptions('quadprog','Algorithm','active-set','Display','off','MaxIterations',500,'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-7);
end
