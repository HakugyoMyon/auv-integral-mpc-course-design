function out=auv12b_surface_align(m,cal,c,a)
% Estimator receives noisy sensors, estimated calibration and noise assumptions only.
assert(~isfield(a,'trueInitialYaw') && ~isfield(a,'coarseYawError') && ~isfield(c,'gyroBias'));
assert(numel(m.gnssTime)>=20 && all(isfinite(m.gnssPosition),'all'));
relYaw=cumtrapz(m.imuTime,m.gyro(:,3)-cal.estimate(3));
relD=interp1(m.imuTime,relYaw,m.dvlTime);omegaD=interp1(m.imuTime,m.gyro(:,3)-cal.estimate(3),m.dvlTime);
scale=[deg2rad(1);deg2rad(.001);1;1];
sigmaBias=sqrt(cal.standard_deviation^2+c.gyroBiasRW^2*a.duration/3);
initialTranslation=m.gnssPosition(1,1:2)'-rot(m.coarseYawStart)*c.rGNSS(1:2);
x0=[0;0;initialTranslation];lb=[-10;-5;-Inf;-Inf];ub=[10;5;Inf;Inf];
options=optimoptions('lsqnonlin','Display','off','MaxIterations',80,'FunctionTolerance',1e-10,'StepTolerance',1e-10);
[x,resnorm,residual,flag,output,~,J]=lsqnonlin(@objective,x0,lb,ub,options);
[y,predGNSS]=prediction(x);covX=pinv(full(J'*J));G=zeros(4);
for k=1:4
 step=zeros(4,1);step(k)=1e-4;yp=prediction(x+step);ym=prediction(x-step);G(:,k)=(yp-ym)/(2e-4);
end
covariance=G*covX*G';Q=c.gyroBiasRW^2;T=m.completionTime;
covariance(1:2,1:2)=covariance(1:2,1:2)+(a.commonGNSSStd^2+a.positionProcessStd^2)*eye(2);
covariance(3:4,3:4)=covariance(3:4,3:4)+auv12b_gyro_process_covariance(T,c.gyroDensity,c.gyroBiasRW)+diag([a.headingFloor^2,0]);
covariance=(covariance+covariance')/2;
rmse=sqrt(mean((predGNSS-m.gnssPosition(:,1:2)).^2,'all'));
valid=flag>0 && all(isfinite(y)) && sqrt(covariance(3,3))<=a.maxHeadingStd && rmse<=a.maxGNSSResidual && cond(full(J'*J))<1e9;
out=struct('valid',valid,'bodyPositionNE',y(1:2),'yaw',y(3),'gyroBiasZ',y(4),'poseBiasCovariance',covariance,...
 'headingStd',sqrt(covariance(3,3)),'gnssFitRMSE',rmse,'parameters',x,'exitflag',flag,'iterations',output.iterations,'resnorm',resnorm,...
 'completionTime',T,'estimatedGNSS',predGNSS);
 function r=objective(z)
  [~,gp]=prediction(z);difference=(gp-m.gnssPosition(:,1:2))/a.fitGNSSStd;
  r=[difference(:);z(1)*scale(1)/a.coarseYawStd;z(2)*scale(2)/max(sigmaBias,1e-12)];
 end
 function [y,gp]=prediction(z)
  delta=z(1)*scale(1);bg=z(2)*scale(2);psi=m.coarseYawStart+delta+relD-bg*m.dvlTime;
  omega=omegaD-bg;vb=m.dvlVelocity(:,1:2)-[-omega*c.rDVL(2),omega*c.rDVL(1)];
  vn=[cos(psi).*vb(:,1)-sin(psi).*vb(:,2),sin(psi).*vb(:,1)+cos(psi).*vb(:,2)];
  path=cumtrapz(m.dvlTime,vn);pGnss=interp1(m.dvlTime,path,m.gnssTime)+z(3:4)';
  psiG=interp1(m.dvlTime,psi,m.gnssTime);gp=pGnss+[cos(psiG)*c.rGNSS(1)-sin(psiG)*c.rGNSS(2),sin(psiG)*c.rGNSS(1)+cos(psiG)*c.rGNSS(2)];
  endYaw=m.coarseYawStart+delta+relYaw(end)-bg*m.completionTime;
  endPosition=path(end,:)'+z(3:4)+rot(psi(end))*vb(end,:)'*(m.completionTime-m.dvlTime(end));
  y=[endPosition;endYaw;cal.estimate(3)+bg];
 end
 function R=rot(psi),R=[cos(psi),-sin(psi);sin(psi),cos(psi)];end
end
