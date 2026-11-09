function tasks=d11_task_examples()
chart=struct('operatingBounds',[-60,400,-150,200],'knownCircles',zeros(0,3),'clearance_m',7);
depth=struct('transit_m',14,'return_m',12,'workMode','altitude','workAltitude_m',8,'maximumRate_m_s',.06);
payload=struct('model','geometric_downlooking_observation_cone','crossHalfAngle_rad',deg2rad(57.5),...
 'alongHalfAngle_rad',deg2rad(30),'maximumRange_m',20,'nominalNavigationMargin_m',1.2,'gridResolution_m',2,'minimumObservations',3);
base=struct('schema','auv.actual_task.v1','id','example','type','survey','startPose',[0,0,0],'homePose',[0,0,0],...
 'speed_m_s',.8,'minimumTurnRadius_m',25,'departureLength_m',20,'maximumTime_s',1200,'chart',chart,'depth',depth,'payload',payload);
a=base;a.id='survey_60x24';a.survey=struct('centerNE',[100,35],'length_m',60,'width_m',24,'bearing_rad',0,'maximumLineSpacing_m',18,'requiredCoverage',.95);
b=base;b.id='pipeline_60m';b.type='inspection';b.inspection=struct('polylineNE',[70,30;130,30],'pointSpacing_m',10,'minimumObservations',3);
c=base;c.id='three_depth_visits';c.type='visit';c.depth.workMode='depth';c.visit=struct('pointsNED',[60,20,15;110,50,16;90,90,13],...
 'priority',[1;2;1],'captureRadius_m',3,'maximumBoundDistance_m',5,'depthTolerance_m',.5,'hoverRequired',false);
tasks={a,b,c};
end
