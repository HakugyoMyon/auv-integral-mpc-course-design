function nav=dyn_nav_adapter(state,insLever,sampleTime,availableTime)
% Adapt an ESKF state at the sonar sampling time, not the current-time state.
R=auv6_rotation(state.q);nav=dyn_contract('navigation');
nav.time=sampleTime;nav.availableTime=availableTime;nav.positionNED=state.p-R*insLever;nav.quaternion=state.q;
S=[0,-insLever(3),insLever(2);insLever(3),0,-insLever(1);-insLever(2),insLever(1),0];
H=zeros(3,15);H(:,1:3)=eye(3);H(:,7:9)=R*S;P=H*state.P*H';nav.positionCovNE=(P(1:2,1:2)+P(1:2,1:2)')/2;
g=zeros(1,3);eps=1e-6;
for k=1:3
 d=zeros(3,1);d(k)=eps;
 Rp=auv6_rotation(auv6_qmul(state.q,[cos(eps/2);sin(eps/2)*d/eps]));
 Rm=auv6_rotation(auv6_qmul(state.q,[cos(eps/2);-sin(eps/2)*d/eps]));
 a=atan2(Rp(2,1),Rp(1,1));b=atan2(Rm(2,1),Rm(1,1));g(k)=atan2(sin(a-b),cos(a-b))/(2*eps);
end
nav.yawVariance=max(0,g*state.P(7:9,7:9)*g');dyn_validate('navigation',nav);
end
