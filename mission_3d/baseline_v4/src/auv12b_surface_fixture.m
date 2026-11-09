function [m,truth]=auv12b_surface_fixture(c,calibrationTruth,a,stream)
% Independent surface kinematic fixture, not cross-surface hydrodynamics.
t=(0:c.dt:a.duration+a.latency)';u=min(t,a.duration);
yaw=a.trueInitialYaw+a.turnAmplitude*sin(2*pi*u/a.duration).^3;
rate=a.turnAmplitude*3*sin(2*pi*u/a.duration).^2.*cos(2*pi*u/a.duration)*2*pi/a.duration;rate(t>a.duration)=0;
vN=a.speed*cos(yaw);vE=a.speed*sin(yaw);p=[cumtrapz(t,vN),cumtrapz(t,vE)];p=p-p(end,:);
N=numel(t);gyro=zeros(N,3);bias=calibrationTruth;
for sample=1:N
 bias=bias+c.gyroBiasRW*sqrt(c.dt)*randn(stream,3,1);
 gyro(sample,:)=(bias+[0;0;rate(sample)]+c.gyroDensity/sqrt(c.dt)*randn(stream,3,1))';
end
dvlIndex=1:round(a.dvlPeriod/c.dt):round(a.duration/c.dt)+1;td=t(dvlIndex);zd=zeros(numel(td),3);
for sample=1:numel(td)
 j=dvlIndex(sample);R=[cos(yaw(j)),-sin(yaw(j)),0;sin(yaw(j)),cos(yaw(j)),0;0,0,1];
 zd(sample,:)=(R'*[vN(j);vE(j);0]+cross([0;0;rate(j)],c.rDVL)+c.dvlStd*randn(stream,3,1))';
end
gnssIndex=1:round(a.gnssPeriod/c.dt):round(a.duration/c.dt)+1;tg=t(gnssIndex);zg=zeros(numel(tg),3);common=a.commonGNSSStd*randn(stream,2,1);
for sample=1:numel(tg)
 j=gnssIndex(sample);R=[cos(yaw(j)),-sin(yaw(j)),0;sin(yaw(j)),cos(yaw(j)),0;0,0,1];
 zg(sample,:)=([p(j,:)';.1]+R*c.rGNSS+c.gnssStd.*randn(stream,3,1)+[common;0])';
end
m=struct('imuTime',t,'gyro',gyro,'dvlTime',td,'dvlVelocity',zd,...
 'gnssTime',tg,'gnssPosition',zg,'gnssAvailableTime',tg+a.latency,...
 'coarseYawStart',yaw(1)+a.coarseYawError,'coarseYawEnd',yaw(end)+a.coarseYawError,'completionTime',t(end));
truth=struct('finalGyroBias',bias,'commonGNSSBias',common,'finalYaw',yaw(end),'finalBodyPosition',[p(end,:)';.1],...
 'time',t,'yaw',yaw,'bodyPosition',[p,.1*ones(N,1)]);
end
