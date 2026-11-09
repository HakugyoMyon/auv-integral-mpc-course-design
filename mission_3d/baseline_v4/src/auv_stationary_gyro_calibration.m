function [calibration,trueFinalBias]=auv_stationary_gyro_calibration(c,stream)
% Separate prelaunch stationary fixture. No true bias is fed to the estimator.
duration=20;N=round(duration/c.dt);truth=c.gyroBias;samples=zeros(3,N);
for k=1:N
 truth=truth+c.gyroBiasRW*sqrt(c.dt)*randn(stream,3,1);
 samples(:,k)=truth+c.gyroDensity/sqrt(c.dt)*randn(stream,3,1);
end
calibration=struct('duration_s',duration,'estimate',mean(samples,2),...
 'standard_deviation',sqrt(c.gyroDensity^2/duration+c.gyroBiasRW^2*duration/3),...
 'sample_time',(-N:-1)'*c.dt,'gyro_samples',samples');
trueFinalBias=truth;
end
