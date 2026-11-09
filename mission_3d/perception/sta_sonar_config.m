function c=sta_sonar_config()
% Declared geometric 3D scanning-sonar MIL assumption, not existing hardware.
c.schema='auv.static_scanner.v1';c.frame='NED_FRD';
c.azimuth_rad=deg2rad(-65:1:65);c.elevation_rad=deg2rad(-35:1:35);
c.minRange_m=.5;c.maxRange_m=60;c.period_s=1;c.latency_s=.1;
c.rangeErrorBound_m=.03;c.dropoutProbability=.01;c.mountFRD=[.55;0;0];
c.terrainGradientBound=.07;c.terrainStep_m=1;
c.maxFrames=160;c.voxelSize_m=.5;c.maxOccupiedVoxels=750000;
c.minimumGridHalfWidths=[1,2,4,8,16,32,64];
c.scope='Full finite-cell multi-elevation geometric scan. Requires a 3D/pivoting sensor capability NOT established by the existing mechanical model. Bounded range error is an explicit MIL assumption.';
end
