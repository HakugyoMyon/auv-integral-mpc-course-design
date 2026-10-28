function [command,memory,info] = pid_control_step(z,psiReference,memory,C,P,trim,low,rudderOverride)
% Discrete PI+D-on-measurement; common amplitude/rate command regulator.
% Controller order is rudder, stern plane, propeller. Integral units rad,rad,RPM.
psiRate=(z(5)*sin(z(10))+z(6)*cos(z(10)))/cos(z(11));
alpha=1-exp(-C.Ts/C.rateFilterTau);
memory.rateFiltered=memory.rateFiltered+alpha*([psiRate;z(5)]-memory.rateFiltered);
[~,R]=eulerang(z(10),z(11),z(12)); velocity=R*z(1:3);
thetaReference=min(max(C.depthKp*(z(9)-10)+C.depthKd*velocity(3),-deg2rad(10)),deg2rad(10));
errorSignal=[angle(exp(1i*(psiReference-z(12))));thetaReference-z(11);1.5-z(1)];
kp=[C.headingKp;C.pitchKp;C.speedKp]; ki=[C.headingKi;C.pitchKi;C.speedKi];
signControl=[1;-1;1];
raw=trim.a+signControl.*(kp.*errorSignal+memory.integral) ...
 +[-C.headingKd*memory.rateFiltered(1);C.pitchKd*memory.rateFiltered(2);0];
% Optional external yaw command; auxiliary equations remain identical.
if nargin>=8
 raw(1)=rudderOverride;
end
bounded=min(max(raw,P.lower),P.upper);
command=min(max(bounded,memory.previousCommand-P.rate*C.Ts),memory.previousCommand+P.rate*C.Ts);
if ~low
 if nargin>=8, ki(1)=0; end
 memory.integral=memory.integral+C.Ts*(ki.*errorSignal ...
  +C.antiWindupGain*signControl.*(command-raw));
end
if nargin>=8, memory.integral(1)=0; end
memory.previousCommand=command;
info=struct('error',errorSignal,'raw',raw,'command',command, ...
 'thetaReference',thetaReference,'integral',memory.integral, ...
 'amplitudeActive',raw<P.lower-1e-12 | raw>P.upper+1e-12, ...
 'rateActive',abs(command-bounded)>1e-10, ...
 'regulatorActive',abs(command-raw)>1e-10,'lowSpeed',low);
end
