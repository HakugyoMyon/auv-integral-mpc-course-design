function probe=sta_probe_terrain(route,mission,xhat,referenceDepth,targetDepth,map,c)
% Map-only threat screening with the SAME nominal tube model as route_check.
% Planned schedule is geometry, not the actual state's depth/attitude proof;
% the fresh nonlinear MPC and actual-command guards remain indispensable.
assert(numel(xhat)>=7&&all(isfinite(xhat(1:7))));
step=c.sampleStep_m;assert(step>0&&step<=1);
lengths=[route.segments.length];globalStart=sum(lengths(1:mission.segment-1))+mission.progress;
s=unique([0:step:c.threatLookahead_m,c.threatLookahead_m]);
edges=[0,cumsum(lengths)];extra=edges-globalStart;
s=unique([s,extra(extra>=0&extra<=c.threatLookahead_m)]);
isScheduled=isfield(route,'staticAvoidance')&&route.staticAvoidance&&isfield(route,'depthSchedule')...
 &&~(isfield(route,'inheritAltitudeMode')&&route.inheritAltitudeMode);
if isScheduled
 schedule=route.depthSchedule;assert(size(schedule,2)==2&&all(diff(schedule(:,1))>0));
 extra=schedule(:,1)'-globalStart;s=unique([s,extra(extra>=0&extra<=c.threatLookahead_m)]);
 scheduleSlopes=diff(schedule(:,2))./diff(schedule(:,1));maxSlope=max(abs(scheduleSlopes));
else
 delta=targetDepth-referenceDepth;rate=c.maximumDepthRate_m_s/c.maximumSpeed_m_s;
 acquisition=abs(delta)/rate;
 if acquisition>0&&acquisition<c.threatLookahead_m,s=unique([s,acquisition]);end
 maxSlope=0;if delta~=0,maxSlope=rate;end
end
terrainMode=isfield(route,'inheritAltitudeMode')&&route.inheritAltitudeMode;
profileMeta=struct();
if terrainMode
 xy=zeros(numel(s),2);
 for i=1:numel(s),xy(i,:)=d16_path_preview(route,mission.segment,mission.progress,s(i));end
 pc=c;pc.referenceDepth_m=referenceDepth;pc.nominalDepth_m=targetDepth;
 [profileZ,profileSlope,~,profileMeta]=sta_altitude_profile(xy,s(:),xhat,pc);
 maxSlope=max(abs(profileSlope));
end
xs=c.bodyX_m(:);points=zeros(numel(s)*numel(xs),3);depths=zeros(size(s));slopes=depths;
for k=1:numel(s)
 [p,yaw]=d16_path_preview(route,mission.segment,mission.progress,s(k));
 if terrainMode
  z=profileZ(k);slope=profileSlope(k);
 elseif isScheduled
  distance=min(max(globalStart+s(k),schedule(1,1)),schedule(end,1));
  z=interp1(schedule(:,1),schedule(:,2),distance,'linear');
  index=find(distance<schedule(2:end,1)-1e-9,1,'first');
  if isempty(index),index=numel(scheduleSlopes);end
  slope=scheduleSlopes(index);
  if globalStart+s(k)>schedule(end,1),slope=0;end
 else
  z=referenceDepth+sign(delta)*min(abs(delta),rate*s(k));
  slope=0;if s(k)<acquisition-1e-9,slope=sign(delta)*rate;end
 end
 unit=[cos(yaw),sin(yaw),slope];unit=unit/norm(unit);
 ids=(k-1)*numel(xs)+(1:numel(xs));points(ids,:)=[p(:)',z]+xs*unit;
 depths(k)=z;slopes(k)=slope;
end
cover=sqrt(c.bodyCrossRadius_m^2+(max(diff(xs))/2)^2);
navBudget=hypot(c.navRadius95_m,3*c.depthSigma_m);
attitude=max(abs(xs))*2*sin(c.attitudeBudget_rad/2);
sampleCover=step/2*sqrt(1+maxSlope^2);
geometryRotation=max(abs(xs))*step/(2*c.turnRadius_m);
margin=cover+navBudget+c.trackingTube_m+c.residualClearance_m+attitude+sampleCover+geometryRotation;
q=c.queryFcn(map,points,margin);states=reshape(q.state,numel(xs),numel(s));
occupied=any(states==1,1);first=find(occupied,1);distance=Inf;
if ~isempty(first),distance=s(first);end
probe=struct('knownObstacleAhead',any(occupied),'knownOccupiedAhead',any(occupied),...
 'firstOccupiedDistance_m',distance,'occupiedSamples',sum(q.state==1),...
 'unknownSamples',sum(q.state<0),'margin_m',margin,'sampleStep_m',step,...
 'routeGlobalStart_m',globalStart,'scheduledDepth',isScheduled,...
 'sampleDistance_m',s(:),'sampleDepth_m',depths(:),'sampleDepthSlope',slopes(:),...
 'capsuleCoverRadius_m',cover,'navigationBudget_m',navBudget,...
 'sampleCover_m',sampleCover,'geometryRotation_m',geometryRotation,...
 'executionVariant','altitude_plane_nominal_v2','altitudeProfileMeta',profileMeta,...
 'geometryModel','sta_route_check longitudinal covering balls with depth-slope tangent',...
 'scope','Nominal continuous-tube occupied screening with route_check budgets and sampling. Unknown remains unknown, never certified free. Static planned depth schedule is not actual motion; fresh nonlinear MPC and actual-command admission remain mandatory.');
end
