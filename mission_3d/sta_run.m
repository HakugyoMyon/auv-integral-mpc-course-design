function report=sta_run(config,budgetFile,outdir,maximumTime,staticSpec,session)
% Isolated unknown-static-obstacle MIL; frozen v4 and dynamic code unchanged.
assert(strcmp(config.execution.environment,'static_unknown')&&isequal(config.execution.avoidanceEnabled,true));
geometryConfig=config;geometryConfig.execution.environment='obstacle_free';geometryConfig.execution.avoidanceEnabled=false;
[check,plan]=auv_task_validate(geometryConfig);assert(check.valid,strjoin(check.errors));
assert(plan.verticalScreen.referenceSlewScreenComplete,'Altitude missions require explicit public depth.workDepthBounds_m for execution.');
assert(nargin>=5&&nargin<=6);if nargin<6,session=[];end
assert(isempty(session)||isa(session,'AUVTaskSession'),'Invalid interactive session.');
operatorReturn=false;operatorCommands={};taskAcceptanceEnabled=true;
task=plan.task;config.task=task;seed=config.execution.seed;if nargin<4,maximumTime=task.maximumTime_s;end
assert(maximumTime>0&&maximumTime<=task.maximumTime_s);assert(~isfile(fullfile(outdir,'mil.mat'))&&~isfile(fullfile(outdir,'report.json')),'Use a new output directory; existing raw evidence will not be overwritten.');if ~isfolder(outdir),mkdir(outdir);end
activeRoute=plan;nominalRoute=plan;nominalPointer=[];mission=[];patchRounds=0;routeRevision=1;routeLog={};gateLog={};
p=auv6_candidate_config(budgetFile);f=auv6_fin_config();m=d14w_mpc_build(p,f,.8);m.uMin(2:5)=-deg2rad(8);m.uMax(2:5)=deg2rad(8);
c=auv6_nav_config();c.headingAid=d11_heading_aid_config();prep=auv12c_prepare(c,seed,3501+(seed-1101));trim=auv6_full_trim(task.speed_m_s,p,f);
x=trim.state;x(3)=task.depth.transit_m;stream=RandStream('mt19937ar','Seed',seed);stream.State=prep.postCalibrationRandomState;
angles=auv6_euler_state(x);estimated=auv6_from_euler_state([angles(1)+deg2rad(.2);angles(2)-deg2rad(.2);prep.alignment.yaw;zeros(11,1)]);
origin=[prep.alignment.bodyPositionNE;x(3)+c.pressureBias+c.pressureStd*randn(stream)];
dvl=x(8:10)+c.dvlStd*randn(stream,3,1);gyro=prep.sensors.gyro(end,:)';bg=prep.calibration.estimate;bg(3)=prep.alignment.gyroBiasZ;
st=auv12b_nav_initialize(origin,estimated(4:7),dvl,gyro,bg,prep.alignment.poseBiasCovariance,prep.calibration,c,prep.configuration);
st.headingBias=0;st.P=blkdiag(st.P,c.headingAid.commonBiasStd_rad^2);
headingStream=RandStream('mt19937ar','Seed',seed+47000);
truthHeadingBias=c.headingAid.commonBiasStd_rad*randn(headingStream);
truthBG=prep.truth.finalGyroBias;truthBA=c.accelBias;act=x(14:18);memory=d3_initialize_memory(trim.control,m);
command=[auv6_thrust_pwm(trim.control(1),p);trim.control(2:5)];N=floor(maximumTime/c.dt)+1;
if strcmp(plan.targets.kind,'coverage_cells'),targetCount=size(plan.targets.centersNE,1);elseif strcmp(plan.targets.kind,'inspection_observation_points'),targetCount=size(plan.targets.positionsNE,1);else,targetCount=size(plan.targets.positionsNED,1);end;[~,allocationRAM]=feval('memory');estimatedGB=max(.5,1.5*(N*500+ceil(N/10)*(10000+8*targetCount))/2^30);assert(allocationRAM.PhysicalMemory.Available/2^30>=.75+estimatedGB,'Insufficient memory before history allocation.');
imu=zeros(6,N);packets=cell(N,1);L=ceil(N/10);h=struct('t',zeros(L,1),'truth',zeros(L,18),'estimate',zeros(L,18),...
 'reference',zeros(L,15),'command',zeros(L,5),'quality',zeros(L,3),'navigationCovariance',zeros(16,16,L),...
 'segment',zeros(L,1),'progress',zeros(L,1),'crossTrack',zeros(L,1),'timings',zeros(L,1),'QP_success',false(L,1));
baseStep=0;started=false;lastDVL=-Inf;mission=[];headingReference=prep.alignment.yaw;depthReference=origin(3);row=0;
payloadConfig=d11_payload_config(task.payload);altimeterConfig=auv6_altimeter_config();
payloadStream=RandStream('mt19937ar','Seed',seed+58000);altimeterStream=RandStream('mt19937ar','Seed',seed+59000);

payloadLedger=auv_task_ledger([],plan,zeros(18,1),struct('valid',false),0,0,struct('valid',false));
payloadLog={};pendingPayload=[];pendingAltitude=[];altimeterLog={};altitude=struct('valid',false,'time',-Inf,'bottomDepth',NaN);altitudeLog=zeros(L,3);
if strcmp(task.type,'survey'),centers=plan.targets.centersNE;cellRadius=sqrt(2)*plan.targets.cellSize_m/2;cellBearing=task.survey.bearing_rad;
elseif strcmp(task.type,'inspection'),centers=plan.targets.positionsNE;cellRadius=0;cellBearing=0;
else,centers=zeros(0,2);cellRadius=0;cellBearing=0;end
alphaPeak=0;ratePeak=0;clipSteps=0;reverseSteps=0;minimumTerrain=Inf;abort='none';
[~,ram]=feval('memory');assert(ram.PhysicalMemory.Available/2^30>1.5);memoryLog=[0,ram.PhysicalMemory.Available/2^30];

scanner=sta_sensor_config();
if isfield(staticSpec,'scannerOverrides')
 fields=fieldnames(staticSpec.scannerOverrides);for k0=1:numel(fields),scanner.(fields{k0})=staticSpec.scannerOverrides.(fields{k0});end
end
scannerStream=RandStream('mt19937ar','Seed',seed+88000);
staticMap=sta_map_init(staticSpec.launchPrior,scanner);scanLog={};pendingScan=[];planningLog={};staticGateLog={};
minimumObstacleClearance=Inf;offlineCollision=false;forceStaticGate=false;
planner=sta_planner_config();planner.clearHorizon_s=3;planner.residualClearance_m=staticSpec.minimumClearance_m;
if isfield(staticSpec,'plannerOverrides')
 keys=fieldnames(staticSpec.plannerOverrides);for qi=1:numel(keys),planner.(keys{qi})=staticSpec.plannerOverrides.(keys{qi});end
end
h.staticActive=false(L,1);h.staticMapScans=zeros(L,1);h.taskAcceptanceEnabled=true(L,1);h.routeRevision=zeros(L,1);h.taskCounts=zeros(L,numel(payloadLedger.counts));
for j=1:N
 step=j-1;t=step*c.dt;R=auv6_rotation(x(4:7));omega=x(11:13);
 truthBG=truthBG+c.gyroBiasRW*sqrt(c.dt)*randn(stream,3,1);truthBA=truthBA+c.accelBiasRW*sqrt(c.dt)*randn(stream,3,1);
 gyro=omega+truthBG+c.gyroDensity/sqrt(c.dt)*randn(stream,3,1);list={};
 truthHeadingBias=truthHeadingBias+c.headingAid.biasRW_rad_sqrt_s*sqrt(c.dt)*randn(headingStream);
 if mod(step,round(c.headingAid.period_s/c.dt))==0
  trueEuler=auv6_euler_state(x);
  z=trueEuler(3)+truthHeadingBias+c.headingAid.whiteStd_rad*randn(headingStream);
  list{end+1}=struct('type','heading','z',atan2(sin(z),cos(z)),...
   'covariance',c.headingAid.whiteStd_rad^2,'valid',max(abs(trueEuler(1:2)))<=c.headingAid.maximumTilt_rad);
 end
 if mod(step,20)==0,list{end+1}=struct('type','dvl','z',x(8:10)+cross(omega,c.rDVL)+c.dvlStd*randn(stream,3,1),'gyro',gyro,'covariance',c.dvlStd^2*eye(3));end
 if mod(step,10)==0,list{end+1}=struct('type','pressure','z',x(3)+R(3,:)*c.rPressure+c.pressureBias+c.pressureStd*randn(stream),'covariance',c.pressureStd^2);end
 if auv_task_window_active(config.execution,'dvlUnavailableWindows_s',t),list=list(~cellfun(@(packet)strcmp(packet.type,'dvl'),list));end
 packets{j}=list;target=max(0,step-round(c.lag/c.dt));
 if step>=round(c.lag/c.dt)&&~started,st=fuse(st,1);started=true;end
 while baseStep<target,st=d11_nav_predict(st,imu(:,baseStep+1),c);baseStep=baseStep+1;st=fuse(st,baseStep+1);end
 if mod(step,10)==0
  clock=tic;
  now=st;for ii=baseStep:step-1,now=d11_nav_predict(now,imu(:,ii+1),c);end
  xhat=auv6_nav_output(now,gyro,act,c);qualityState=now;qualityState.P=now.P(1:15,1:15);quality=auv12b_nav_quality(qualityState,c,t,lastDVL);
  H=zeros(3,16);H(:,1:3)=eye(3);H(:,7:9)=auv6_rotation(now.q)*auv6_skew(c.rINS);P=H*now.P*H';
  nav=d15_nav_adapter(now,c.rINS,t,t);
  staticNav=sta_nav_contract(xhat,quality,P,now.P(7:9,7:9),t);
  planner.navRadius95_m=staticNav.positionErrorBound_m;planner.depthSigma_m=sqrt(max(0,P(3,3)));
  planner.attitudeBudget_rad=max(deg2rad(2),staticNav.attitudeErrorBound_rad);
  if ~isempty(pendingScan)&&t>=pendingScan.packet.availableTime
   [staticMap,scanEvent]=sta_map_update(staticMap,pendingScan.packet,pendingScan.captureNav);
   pendingScan.event=scanEvent;scanLog{end+1}=pendingScan;pendingScan=[];
  end
  if isempty(pendingScan)&&(mod(step,round(scanner.period_s/c.dt))==0||staticMap.acceptedScans==0)
   pendingScan=struct('packet',sta_sensor(staticSpec.world,x,t,scanner,scannerStream),'captureNav',staticNav);
  end

  operatorCommands={};
  acceptanceThisTick=taskAcceptanceEnabled;
  if ~isempty(session)
   drawnow limitrate;operatorCommands=session.poll(t);
   while session.Paused&&~session.StopRequested
    pause(.05);drawnow;operatorCommands=[operatorCommands,session.poll(t)];
    [~,pauseRAM]=feval('memory');if pauseRAM.PhysicalMemory.Available/2^30<.75,abort='resource_floor';break;end
   end
   if session.StopRequested,abort='operator_stopped_simulation';end
  end
  requestReturn=[];
  for ci=1:numel(operatorCommands)
   ce=operatorCommands{ci};
   if strcmp(ce.kind,'return_home')
    if operatorReturn,session.acknowledge(ce,t,false,'return_already_active');
    elseif isempty(requestReturn),requestReturn=ce;
    else,session.acknowledge(ce,t,false,'duplicate_pending_return');end
   else,session.acknowledge(ce,t,true,'simulation_control_only');end
  end
  if ~isempty(pendingAltitude)&&t>=pendingAltitude.packet.availableTime
   ap=pendingAltitude;estimateAltitude=auv6_altitude_estimate(ap.packet,ap.navigation(4:7),altimeterConfig);
   if estimateAltitude.valid,altitude=struct('valid',true,'time',ap.packet.time,'bottomDepth',ap.navigation(3)+estimateAltitude.altitude);end
   ap.processed=estimateAltitude;altimeterLog{end+1}=ap;pendingAltitude=[];
  end
  if ~isempty(pendingPayload)&&t>=pendingPayload.packet.availableTime
   pp=pendingPayload;
   observation=d11_payload_observe(pp.packet,pp.navigation,pp.quality,pp.attitudeCovariance,centers,cellRadius,payloadConfig,t);
   if ~quality.valid||~altitude.valid||t-altitude.time>.7||abs(altitude.bottomDepth-pp.navigation(3)-task.depth.workAltitude_m)>.75
    observation.valid=false;observation.indices=[];
   end
   pp.taskAcceptanceEnabled=taskAcceptanceEnabled;
   if taskAcceptanceEnabled,payloadLedger=d11_payload_ledger(payloadLedger,plan,observation,quality);end
   pp.observation=observation;pp.acceptedAt=t;pp.acceptanceQuality=quality;pp.heightEvidence=altitude;payloadLog{end+1}=pp;pendingPayload=[];
  end
  if mod(step,20)==0
   altimeterPacket=sta_altimeter(x,t,altimeterConfig,@auv6_terrain,altimeterStream,false,staticSpec.world);
   % Static wrapper removes simulator truth before returning the packet.
   altimeterPacket.forcedUnavailable=auv_task_window_active(config.execution,'altimeterUnavailableWindows_s',t);if altimeterPacket.forcedUnavailable,altimeterPacket.valid=false;altimeterPacket.beamValid(:)=false;end
   pendingAltitude=struct('packet',altimeterPacket,'navigation',xhat);
   if ~auv_task_is_point_task(task.type),pendingPayload=struct('packet',sta_payload_sensor(x,t,payloadConfig,@auv6_terrain,payloadStream,false,staticSpec.world),...
    'navigation',xhat,'quality',quality,'attitudeCovariance',now.P(7:9,7:9),'truthForOfflineAuditOnly',x);pendingPayload.packet.forcedUnavailable=auv_task_window_active(config.execution,'payloadUnavailableWindows_s',t);if pendingPayload.packet.forcedUnavailable,pendingPayload.packet.valid=false;end;end
  end

  if mod(step,1000)==0,[~,ram]=feval('memory');memoryLog(end+1,:)=[t,ram.PhysicalMemory.Available/2^30];if memoryLog(end,2)<.75,abort='resource_floor';break;end;end
  if taskAcceptanceEnabled&&auv_task_is_point_task(task.type),payloadLedger=auv_task_ledger(payloadLedger,plan,xhat,quality,sqrt(max(0,P(3,3))),t,struct());end
  [mission,status]=sta_step(mission,activeRoute,xhat,quality,sqrt(max(0,P(3,3))),t,altitude,payloadLedger);
  forceStaticGate=false;
  localActive=isfield(activeRoute,'staticAvoidance')&&activeRoute.staticAvoidance;
  if localActive&&mission.finished
   routeBack=activeRoute.resumeRoute;pointer=mission;pointer.segment=activeRoute.resumePoint.segment;pointer.progress=activeRoute.resumePoint.progress;
   pointer.projectionMaxDistance_m=3;pointer.projectionWindow_m=20;
   [pointer,projectionAudit]=sta_project_master(routeBack,xhat,pointer);
   if ~projectionAudit.accepted,abort='static_resume_projection_rejected';
   else
    pointer.finished=false;pointer.complete=false;pointer.returnVerified=false;pointer.terminalVerified=false;pointer.terminalApproachActive=false;
    activeRoute=routeBack;nominalRoute=routeBack;mission=pointer;routeRevision=routeRevision+1;forceStaticGate=true;
    routeLog{end+1}=struct('time',t,'kind','static_resume','route',activeRoute,'revision',routeRevision,'appliedAt',t,'reason','estimated_pose_reprojection');
    [mission,status]=sta_step(mission,activeRoute,xhat,quality,sqrt(max(0,P(3,3))),t,altitude,payloadLedger);
   end
  end
  localActive=isfield(activeRoute,'staticAvoidance')&&activeRoute.staticAvoidance;
  if ~localActive,nominalPointer=mission;end
  if posixtime(datetime('now','TimeZone','UTC'))>=staticSpec.computeStopPosix,abort='work_time_limit';end
  if isfile(fullfile(outdir,'OWNER_STOP_REQUEST.txt')),abort='owner_stopped_test';end
  if offlineCollision,abort='offline_truth_collision_observer';end
  if staticMap.capacityExceeded,abort='static_map_capacity_limit';end
  if status.heightMissing,abort='altimeter_unavailable_during_altitude_work';end
  if status.outsideDepthEnvelope,abort='height_target_outside_declared_work_depth';end
  if t>.1&&(~quality.valid||quality.radius95>5),abort='navigation_quality_invalid';end
  if status.returnConfirmationRejected,abort='return_not_confirmed_at_route_end';end
  stagedRoute=[];stagedMission=[];routeKind='remaining_task_route';
  if mod(step,50)==0&&~mission.finished&&strcmp(abort,'none')
   planner.referenceDepth_m=depthReference;planner.nominalDepth_m=status.depthTarget;
   threat=sta_probe_consistent(activeRoute,mission,xhat,depthReference,status.depthTarget,staticMap,planner);
   if threat.knownObstacleAhead
    [replanMaster,replanPointer,anchorAudit]=sta_replan_anchor(activeRoute,mission,xhat);
    proposal=sta_local_plan_lazy(xhat,replanMaster,replanPointer,staticMap,planner);
    proposal.anchorAudit=anchorAudit;
    if proposal.accepted,proposal.timeBudget=sta_time_budget(proposal.route,payloadLedger,t,task.maximumTime_s);end
    planningLog{end+1}=struct('time',t,'threat',threat,'proposal',proposal,'mapScans',staticMap.acceptedScans,'navigation',staticNav);
    if proposal.accepted
     stagedRoute=proposal.route;stagedMission=mission;stagedMission.segment=1;stagedMission.progress=0;
     stagedMission.finished=false;stagedMission.complete=false;stagedMission.returnVerified=false;stagedMission.terminalVerified=false;stagedMission.terminalApproachActive=false;
     routeKind='static_detour';recoveryReason='observed_static_threat';
     [stagedMission,status]=sta_step(stagedMission,stagedRoute,xhat,quality,sqrt(max(0,P(3,3))),t,altitude,payloadLedger);
    else,abort='static_no_map_certified_candidate';end
   end
  end
  if ~isempty(requestReturn)&&strcmp(abort,'none')
   [stagedRoute,recoveryReason]=auv_task_operator_return(plan,nav,t,maximumTime);
   routeKind='operator_return';
   if isempty(stagedRoute),session.acknowledge(requestReturn,t,false,recoveryReason);
   else
    stagedMission=[];[stagedMission,status]=sta_step(stagedMission,stagedRoute,xhat,quality,sqrt(max(0,P(3,3))),t,altitude,payloadLedger);
   end
  end
  if mission.finished&&~mission.complete&&~operatorReturn&&isempty(stagedRoute)&&strcmp(abort,'none')
   if patchRounds<3,[stagedRoute,recoveryReason]=auv_task_recovery_plan(plan,payloadLedger,nav,t);else,recoveryReason='recovery_round_cap';end
   if isempty(stagedRoute),abort=['incomplete_task:',recoveryReason];
   else
    stagedMission=mission;stagedMission.segment=1;stagedMission.progress=0;stagedMission.finished=false;stagedMission.complete=false;stagedMission.returnVerified=false;
    [stagedMission,status]=sta_step(stagedMission,stagedRoute,xhat,quality,sqrt(max(0,P(3,3))),t,altitude,payloadLedger);
   end
  end
  if isempty(stagedRoute),[mission,ref,ur,nextHeading,nextDepth]=d16_guidance(mission,activeRoute,xhat,headingReference,depthReference,m,trim,status);
  else,[stagedMission,ref,ur,nextHeading,nextDepth]=d16_guidance(stagedMission,stagedRoute,xhat,headingReference,depthReference,m,trim,status);end
  if (mod(step,50)==0||~isempty(stagedRoute)||forceStaticGate)&&(~mission.finished||~isempty(stagedRoute))&&strcmp(abort,'none')
   context=struct('schema','auv.execution_state.v1','revision',row+1,'time',t,'state',xhat,...
    'commandHistoryValid',true,'lastCommandTime',max(0,t-m.Ts),'actuatorTime',t,'actuatorSource','command_model',...
    'flowAssumption','still_water','finHealth',ones(4,1),'thrustHealth',1,'actuatorUncertainty_deg',.05,'memory',memory);
   gate=d72_fresh_gate(context,ref,ur,task.speed_m_s,m,p,f,.05);
   if gate.accepted
    mapAdmission=sta_trajectory_query(gate.predictedTrajectory,staticMap,staticNav,planner,3);
    staticGateLog{end+1}=struct('time',t,'admission',mapAdmission,'mapScans',staticMap.acceptedScans,'navigation',staticNav);
    if ~mapAdmission.accepted,gate.accepted=false;gate.reason='static_MPC_prefix_not_observed_free';end
   end
   recordContext=context;recordGate=gate;
   if isfield(recordContext.memory,'solverWarmStart'),recordContext.memory=rmfield(recordContext.memory,'solverWarmStart');end
   if isfield(recordGate,'nextMemory')&&isstruct(recordGate.nextMemory)&&isfield(recordGate.nextMemory,'solverWarmStart'),recordGate.nextMemory=rmfield(recordGate.nextMemory,'solverWarmStart');end
   gateLog{end+1}=struct('time',t,'context',recordContext,'reference',ref,'inputReference',ur,'gate',recordGate);
   if gate.accepted,u=gate.firstCommand;memory=gate.nextMemory;info=gate.currentInfo;
    if ~isempty(stagedRoute)
     activeRoute=stagedRoute;mission=stagedMission;routeRevision=routeRevision+1;
     if strcmp(routeKind,'operator_return'),operatorReturn=true;taskAcceptanceEnabled=false;session.acknowledge(requestReturn,t,true,'fresh_MPC_admitted');
     elseif strcmp(routeKind,'remaining_task_route'),patchRounds=patchRounds+1;nominalRoute=activeRoute;else,nominalRoute=activeRoute.resumeRoute;end
     appliedAt=NaN;if j<N,appliedAt=t;end
     routeLog{end+1}=struct('time',t,'kind',routeKind,'route',activeRoute,'revision',routeRevision,'appliedAt',appliedAt,'reason',recoveryReason);
    end
   else,if ~isempty(requestReturn),session.acknowledge(requestReturn,t,false,['fresh_MPC_rejected:',gate.reason]);end;abort=['physical_preview:',gate.reason];u=m.u0+m.scale.*memory.previousU;info=struct('success',gate.QP_success);end
  elseif ~mission.finished&&strcmp(abort,'none')
   [u,memory,info]=d14w_mpc_continuous_step(auv_mpc_state(xhat),ref,ur,m,memory);
  else,u=m.u0+m.scale.*memory.previousU;info=struct('success',true);end
  headingReference=nextHeading;depthReference=nextDepth;
  [runtimeOkay,~,runtimeState]=d15_runtime_preview(xhat,u,p,f);if ~runtimeOkay,abort='runtime_physical_guard';end
  if strcmp(abort,'none')
   runtimeMap=sta_trajectory_query([xhat,runtimeState],staticMap,staticNav,planner,.1);
   if ~runtimeMap.accepted,abort='static_actual_command_prefix_not_free';end
  end
  if ~info.success,abort='QP_failure';end
  command=[auv6_thrust_pwm(u(1),p);u(2:5)];row=row+1;
  h.t(row)=t;h.truth(row,:)=x';h.estimate(row,:)=xhat';h.reference(row,:)=ref(:,1)';h.command(row,:)=u';h.quality(row,:)=[quality.radius95,quality.yawStd,quality.dvlAge];
  h.navigationCovariance(:,:,row)=now.P;h.segment(row)=mission.segment;h.progress(row)=mission.progress;h.crossTrack(row)=status.crossTrack;h.timings(row)=toc(clock);h.QP_success(row)=info.success;
  h.staticActive(row)=isfield(activeRoute,'staticAvoidance')&&activeRoute.staticAvoidance;h.staticMapScans(row)=staticMap.acceptedScans;
  h.taskAcceptanceEnabled(row)=acceptanceThisTick;h.routeRevision(row)=routeRevision;h.taskCounts(row,:)=payloadLedger.counts';altitudeLog(row,:)=[status.workMode,altitude.bottomDepth-xhat(3),status.depthTarget];
  if ~isempty(session),session.publish(sta_live_status(t,xhat,sum(payloadLedger.counts>=payloadLedger.minimumObservations),numel(payloadLedger.counts),operatorReturn,abort,h.staticActive(row),staticMap));end
  if mission.finished||~strcmp(abort,'none'),break;end
  if mod(step,1000)==0,fid=fopen(fullfile(outdir,'progress.txt'),'w');fprintf(fid,'t=%.1f type=%s segment=%d/%d completed=%d/%d N=%.2f E=%.2f depth=%.2f',t,task.type,mission.segment,numel(activeRoute.segments),sum(payloadLedger.counts>=payloadLedger.minimumObservations),numel(payloadLedger.counts),x(1),x(2),x(3));fclose(fid);end
 end
 if j==N,break;end
 trueClearance=sta_world_clearance(x,staticSpec.world);minimumObstacleClearance=min(minimumObstacleClearance,trueClearance);
 offlineCollision=offlineCollision||trueClearance<0;
 [a,aux]=auv6_actuated_rhs(x,command,p,f);[b,auxB]=auv6_actuated_rhs(x+c.dt*a/2,command,p,f);[d,auxD]=auv6_actuated_rhs(x+c.dt*b/2,command,p,f);[e,auxE]=auv6_actuated_rhs(x+c.dt*d,command,p,f);
 alphaPeak=max([alphaPeak;abs(aux.fin.alpha);abs(auxB.fin.alpha);abs(auxD.fin.alpha);abs(auxE.fin.alpha)]);
 clipSteps=clipSteps+double(any(abs(aux.fin.alpha)>f.alphaLimit));reverseSteps=reverseSteps+double(aux.fin.reverseFlowUnsupported);ratePeak=max(ratePeak,max(abs(a(15:18))));minimumTerrain=min(minimumTerrain,auv6_clearance_bound(x,@auv6_terrain));
 specific=a(8:10)+cross(omega,x(8:10))+cross(a(11:13),c.rINS)+cross(omega,cross(omega,c.rINS))-R'*c.gravity;
 imu(:,j)=[gyro;specific+truthBA+c.accelDensity/sqrt(c.dt)*randn(stream,3,1)];
 targetThrust=interp1(p.pwm,p.staticThrust,command(1),'linear');if abs(command(1)-1500)<=25,targetThrust=0;end
 act=d16_actuator_update(act,command,p,f,c.dt);
 x=x+c.dt/6*(a+2*b+2*d+e);x(4:7)=x(4:7)/norm(x(4:7));
end
names=fieldnames(h);for k=1:numel(names),value=h.(names{k});if strcmp(names{k},'navigationCovariance'),h.(names{k})=value(:,:,1:row);else,h.(names{k})=value(1:row,:);end;end
h.altitude=altitudeLog(1:row,:);h.appliedCommandDuration_s=[diff(h.t);max(0,t-h.t(end))];h.terminalPhysicalState=x;h.terminalTime_s=t;h.imu=imu(:,1:max(0,j-1));h.packets=packets(1:j);h.initialAppliedControl=trim.control;
if ~mission.finished&&strcmp(abort,'none'),abort='time_horizon_reached';end
[terminalGoal,terminalKind]=auv_task_terminal(activeRoute);
report=struct('task',task.id,'type',task.type,'environment','static_unknown','duration_s',t,'onboard_task_complete',mission.complete,...
 'return_verified',mission.returnVerified,'confirmed_targets',sum(payloadLedger.counts>=payloadLedger.minimumObservations),'total_targets',numel(payloadLedger.counts),...
 'true_home_error_m',norm(x(1:2)'-task.homePose(1:2)),'true_return_depth_error_m',abs(x(3)-task.depth.return_m),...
 'maximum_raw_alpha_deg',rad2deg(alphaPeak),'clipped_steps',clipSteps,'reverse_flow_steps',reverseSteps,'maximum_fin_rate_deg_s',rad2deg(ratePeak),...
 'minimum_terrain_clearance_m',minimumTerrain,'QP_failures',sum(~h.QP_success),'abort',abort,'physical_and_task_pass',false,...
 'scope','Unified basic-task MIL candidate. Raw truth/task audit and full mission not yet qualified; simulation stop is not physical safe stop.');
[trueCounts,falseClaims]=auv_task_truth_counts(plan,payloadLedger,h,payloadLog,payloadConfig);
report.terminal_verified=mission.terminalVerified;report.terminal_kind=terminalKind;report.true_terminal_error_m=norm(x(1:2)-terminalGoal(1:2));report.true_terminal_depth_error_m=abs(x(3)-terminalGoal(3));
report.true_target_confirmations=sum(trueCounts>=payloadLedger.minimumObservations);report.false_confirmations=falseClaims;
report.physical_and_task_pass=mission.complete&&mission.terminalVerified&&strcmp(abort,'none')&&falseClaims==0&&report.true_terminal_error_m<=5&&report.true_terminal_depth_error_m<=.5&&alphaPeak<=deg2rad(15)&&clipSteps==0&&reverseSteps==0&&minimumTerrain>=2&&all(h.QP_success);
report.operator_return_requested_and_admitted=operatorReturn;
report.operator_return_complete=operatorReturn&&mission.returnVerified&&strcmp(abort,'none')&&falseClaims==0&&report.true_terminal_error_m<=5&&report.true_terminal_depth_error_m<=.5&&alphaPeak<=deg2rad(15)&&clipSteps==0&&reverseSteps==0&&minimumTerrain>=2&&all(h.QP_success);
report.minimum_obstacle_clearance_m=minimumObstacleClearance;
report.map_query_policy=func2str(planner.queryFcn);report.world_kind=staticSpec.kind;report.world_seed=staticSpec.seed;
report.maximum_nominal_candidate_length_m=planner.maximumRouteLength_m;report.helical_candidates_enabled=planner.enableHelical;
report.simulated_sensor_assumption=scanner.scope;report.numeric_threads=maxNumCompThreads;
report.static_avoidance_switches=sum(cellfun(@(event)strcmp(event.kind,'static_detour'),routeLog));
report.static_scans_accepted=staticMap.acceptedScans;report.map_guard_voxels=size(staticMap.occupiedCentersNED,1);
report.physical_and_task_pass=report.physical_and_task_pass&&minimumObstacleClearance>=staticSpec.minimumClearance_m;
report.scope='Static unknown3D obstacle MIL candidate, with explicit scanning/launch prior/uncertainty assumptions. Truth used only by sensors and offline failure/clearance observer. Complete raw and obstacle audits required.';
if isempty(session),interactionLog={};else,interactionLog=session.Log;session.finish(report);end
save(fullfile(outdir,'mil.mat'),'h','plan','activeRoute','mission','config','payloadLedger','payloadLog','payloadConfig','altimeterConfig','altimeterLog','p','f','m','c','trim','prep','gateLog','memoryLog','routeLog','report','interactionLog','staticSpec','staticMap','scanLog','scanner','planningLog','staticGateLog','planner','-v7.3');
fid=fopen(fullfile(outdir,'report.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
 function value=fuse(value,index)
  for iz=1:numel(packets{index})
   [value,info]=d11_nav_update(value,packets{index}{iz},c);
   if info.accepted&&strcmp(packets{index}{iz}.type,'dvl'),lastDVL=(index-1)*c.dt;end
  end
 end
end


