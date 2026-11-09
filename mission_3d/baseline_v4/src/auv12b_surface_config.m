function a=auv12b_surface_config()
a.duration=120;a.gnssPeriod=1;a.dvlPeriod=.2;a.latency=.1;a.speed=.8;
a.turnAmplitude=deg2rad(10);a.trueInitialYaw=0;a.coarseYawError=deg2rad(1.5);
a.coarseYawStd=deg2rad(1);a.commonGNSSStd=.3;
a.fitGNSSStd=.65;a.positionProcessStd=.05;a.headingFloor=deg2rad(.1);
a.maxHeadingStd=deg2rad(.5);a.maxGNSSResidual=1.3;
end
