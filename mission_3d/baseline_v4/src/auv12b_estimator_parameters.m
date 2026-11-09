function [n,e]=auv12b_estimator_parameters(c,a)
% Explicit allowlist: do not pass simulated true heading/bias into alignment.
n=struct('gyroBiasRW',c.gyroBiasRW,'gyroDensity',c.gyroDensity,'rDVL',c.rDVL,'rGNSS',c.rGNSS);
e=struct('duration',a.duration,'fitGNSSStd',a.fitGNSSStd,'coarseYawStd',a.coarseYawStd,...
 'commonGNSSStd',a.commonGNSSStd,'positionProcessStd',a.positionProcessStd,'headingFloor',a.headingFloor,...
 'maxHeadingStd',a.maxHeadingStd,'maxGNSSResidual',a.maxGNSSResidual);
end
