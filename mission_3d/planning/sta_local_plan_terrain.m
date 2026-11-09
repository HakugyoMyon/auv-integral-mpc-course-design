function proposal=sta_local_plan_terrain(xhat,master,mission,map,c)
% Finite 3-D maneuver catalog from public navigation/map inputs ONLY.
% accepted means geometric/observed-prefix candidate, NOT MPC admission.
Rbody=auv6_rotation(xhat(4:7));yaw=atan2(Rbody(2,1),Rbody(1,1));start=[xhat(1:2);yaw];
assert(numel(xhat)==18&&all(isfinite(xhat))&&c.turnRadius_m>=25);
assert(c.maximumDepthRate_m_s>0&&c.maximumDepthRate_m_s<=.06);
empty=master.segments([]);records={};routes={};
holds=c.sideHold_m;
if isfield(c,'sideHoldCandidates_m'),holds=unique([holds,c.sideHoldCandidates_m(:)'],'stable');end
assert(isnumeric(holds)&&all(isfinite(holds))&&all(holds>0));
% A direct continuation is useful for initial trigger/gating and recovery.
[goal,heading,joinIndex,joinProgress]=d16_path_preview(master,mission.segment,mission.progress,c.joinAdvance_m);
[parts,connection]=d15_connect(start,[goal(:);heading],c.turnRadius_m,master.task.chart,'static_avoidance',0);
if connection.accepted
 schedule=flatSchedule(parts,c.referenceDepth_m,c.nominalDepth_m,c);
 addCandidate(parts,schedule,repmat(c.nominalDepth_m,1,numel(parts)),joinIndex,joinProgress,'nominal',0,0,0);
end
for side=[-1,1]
 if strcmp(c.preferredAxis,'vertical'),continue;end
 for angle=c.turnAngles_deg
  for holdLength=holds
  theta=deg2rad(angle);pose=start;parts=empty;
  directions=[side,-side,0,-side,side];lengths=[c.turnRadius_m*theta,c.turnRadius_m*theta,holdLength,c.turnRadius_m*theta,c.turnRadius_m*theta];
  for k=1:5
   if directions(k)==0,p=d11_segment(pose,'line',lengths(k),Inf,0,'static_avoidance',0);
   else,p=d11_segment(pose,'arc',lengths(k),c.turnRadius_m,directions(k),'static_avoidance',0);end
   parts=[parts,p];pose=[p.finish;p.heading1];
  end
  forward=4*c.turnRadius_m*sin(theta)+holdLength;
  [parts,joined,index,progress]=appendJoin(parts,pose,max(c.joinAdvance_m,forward+10));
  if ~joined,continue;end
  schedule=flatSchedule(parts,c.referenceDepth_m,c.nominalDepth_m,c);
  addCandidate(parts,schedule,repmat(c.nominalDepth_m,1,numel(parts)),index,progress,'horizontal',side,angle,holdLength);
  end
 end
end
for vertical=[-1,1]
 if strcmp(c.preferredAxis,'horizontal'),continue;end
 for offset=c.depthOffsets_m
  escape=c.referenceDepth_m+vertical*offset;
  if escape<c.depthBounds_m(1)||escape>c.depthBounds_m(2),continue;end
  acquire=(abs(escape-c.referenceDepth_m)/c.maximumDepthRate_m_s+5)*c.maximumSpeed_m_s;
  hold=max(c.verticalHold_m,c.threatLookahead_m-acquire);
  restore=(abs(c.nominalDepth_m-escape)/c.maximumDepthRate_m_s+5)*c.maximumSpeed_m_s;
  lengths=[acquire,hold,restore];pose=start;parts=empty;
  for k=1:3
   p=d11_segment(pose,'line',lengths(k),Inf,0,'static_avoidance',0);parts=[parts,p];pose=[p.finish;p.heading1];
  end
  mainLength=sum(lengths);[parts,joined,index,progress]=appendJoin(parts,pose,max(c.joinAdvance_m,mainLength+5));
  if ~joined,continue;end
  total=sum([parts.length]);schedule=[0,c.referenceDepth_m;acquire,escape;acquire+hold,escape;mainLength,c.nominalDepth_m];
  if total>mainLength+1e-8,schedule(end+1,:)=[total,c.nominalDepth_m];end
  targets=[escape,escape,repmat(c.nominalDepth_m,1,numel(parts)-2)];
  addCandidate(parts,schedule,targets,index,progress,'vertical',vertical,offset,hold);
 end
end
% Hierarchical feasibility policy: evaluate helices only after standard failure.
% This intentionally does NOT optimize globally over standard plus helical costs.
standardFeasible=any(cellfun(@(a)a.accepted&&~a.knownOccupiedCollision,records));
% Optional depth-acquisition circle, inspired by Dubins airplane helices.
% Not an observed-safe loiter guarantee: every committed prefix still needs
% the actual scan-map and nonlinear MPC gate. Disabled for existing scenarios.
if ~standardFeasible&&isfield(c,'enableHelical')&&c.enableHelical
 offsets=6.5;if isfield(c,'helicalDepthOffsets_m'),offsets=c.helicalDepthOffsets_m;end
 assert(all(isfinite(offsets))&&all(offsets>0));
 if ~strcmp(c.preferredAxis,'horizontal')
  for vertical=[-1,1]
   for offset=offsets(:)'
    escape=c.referenceDepth_m+vertical*offset;
    if escape<c.depthBounds_m(1)||escape>c.depthBounds_m(2),continue;end
    circleLength=2*pi*c.turnRadius_m;
    acquire=(abs(escape-c.referenceDepth_m)/c.maximumDepthRate_m_s+5)*c.maximumSpeed_m_s;
    if acquire>circleLength,continue;end
    lead=15;if isfield(c,'helicalLeadIn_m'),lead=c.helicalLeadIn_m;end
    assert(isfinite(lead)&&lead>0&&lead<c.threatLookahead_m+10);
    hold=c.threatLookahead_m+10-lead;
    restore=(abs(c.nominalDepth_m-escape)/c.maximumDepthRate_m_s+5)*c.maximumSpeed_m_s;
    for direction=[-1,1]
     pose=start;parts=empty;
     p=d11_segment(pose,'line',lead,Inf,0,'static_avoidance',0);
     parts=[parts,p];pose=[p.finish;p.heading1];
     p=d11_segment(pose,'arc',circleLength,c.turnRadius_m,direction,'static_avoidance',0);
     parts=[parts,p];pose=[p.finish;p.heading1];
     p=d11_segment(pose,'line',hold,Inf,0,'static_avoidance',0);
     parts=[parts,p];pose=[p.finish;p.heading1];
     p=d11_segment(pose,'line',restore,Inf,0,'static_avoidance',0);
     parts=[parts,p];pose=[p.finish;p.heading1];
     mainLength=lead+circleLength+hold+restore;
     [parts,joined,index,progress]=appendJoin(parts,pose,max(c.joinAdvance_m,lead+hold+restore+10));
     if ~joined,continue;end
     total=sum([parts.length]);
     schedule=[0,c.referenceDepth_m;lead,c.referenceDepth_m;lead+acquire,escape;lead+circleLength+hold,escape;mainLength,c.nominalDepth_m];
     if total>mainLength+1e-8,schedule(end+1,:)=[total,c.nominalDepth_m];end
     targets=[c.referenceDepth_m,escape,escape,repmat(c.nominalDepth_m,1,numel(parts)-3)];
     addCandidate(parts,schedule,targets,index,progress,'helical_climb_then_pass',vertical,offset,hold,direction);
    end
   end
  end
 end
end
best=0;cost=Inf;
for k=1:numel(records)
 a=records{k};if ~a.accepted||a.knownOccupiedCollision,continue;end
 % Prefer straight nominal only if its whole observed threat corridor is clear.
 if strcmp(a.kind,'nominal')&&any(a.sampleState(a.sample.s_m<=c.threatLookahead_m)==1),continue;end
 if a.score<cost,cost=a.score;best=k;end
end
proposal=struct('accepted',best>0,'route',[],'candidates',{records},...
 'selectedIndex',best,'prefixRequiredDistance_m',c.maximumSpeed_m_s*c.clearHorizon_s,...
 'commitDistance_m',0,'reason','no_map_certified_maneuver_candidate',...
 'scope','Public-map finite maneuver candidates; full tail may be unknown and is uncommitted. Requires fresh MPC/actuator admission and rolling prefix recertification. No stopping, hovering or global completeness guarantee.');
proposal.searchPolicy='hierarchical_standard_then_helical';
proposal.executionVariant='altitude_plane_nominal_v2';
proposal.standardFeasible=standardFeasible;
proposal.helicalCatalogEvaluated=~standardFeasible&&isfield(c,'enableHelical')&&c.enableHelical;
if best>0
 proposal.route=routes{best};proposal.commitDistance_m=records{best}.commitDistance_m;
 proposal.reason='candidate_requires_fresh_MPC';
end

 function [parts,joined,index,progress]=appendJoin(parts,pose,advance)
  [point,gamma,index,progress]=d16_path_preview(master,mission.segment,mission.progress,advance);point=point(:);joined=true;
  if norm(pose(1:2)-point)<1e-6&&abs(atan2(sin(pose(3)-gamma),cos(pose(3)-gamma)))<1e-6,return;end
  [tail,audit]=d15_connect(pose,[point;gamma],c.turnRadius_m,master.task.chart,'static_avoidance',0);
  if ~audit.accepted,joined=false;return;end
  parts=[parts,tail];
 end
 function addCandidate(parts,schedule,targets,index,progress,kind,side,amount,holdLength,turnDirection)
  if nargin<10,turnDirection=0;end
  if isempty(parts)||sum([parts.length])>c.maximumRouteLength_m||~d11_chart_clear(parts,master.task.chart),return;end
  route=master;route.segments=parts;route.depthTargets=targets(:)';route.depthSchedule=schedule;
  [route.inheritAltitudeMode,route.altitudeDepthBounds_m]=sta_altitude_inheritance(master,mission,kind,c);
  route.executionVariant='altitude_plane_nominal_v2';
  if route.inheritAltitudeMode
   sampled=sta_segments_sample(parts,schedule,c.sampleStep_m);
   [~,~,heightSchedule,heightMeta]=sta_altitude_profile(sampled.pointsNED(:,1:2),sampled.s_m,xhat,c);
   route.depthSchedule=heightSchedule;route.altitudeProfileMeta=heightMeta;
  end
  route.resumeRoute=master;route.resumePoint=struct('segment',index,'progress',progress);
  route.originPointer=struct('segment',mission.segment,'progress',mission.progress);
  route.staticAvoidance=true;route.totalLength_m=sum([parts.length]);route.maneuverKind=kind;route.maneuverSide=side;
  route.helicalTurnDirection=turnDirection;
  audit=sta_route_check(route,map,c);audit.kind=kind;audit.side=side;audit.amount=amount;audit.length_m=route.totalLength_m;audit.hold_m=holdLength;
  audit.turnDirection=turnDirection;
  preference=0;if c.preferredSide~=0&&side~=0&&side~=c.preferredSide,preference=20;end
  audit.score=route.totalLength_m+preference+10*audit.unknownSamples/max(1,numel(audit.sampleState));
  records{end+1}=audit;routes{end+1}=route;
 end
end
function schedule=flatSchedule(parts,initial,target,c)
total=sum([parts.length]);required=abs(target-initial)/c.maximumDepthRate_m_s*c.maximumSpeed_m_s;
if required>0&&required<total,schedule=[0,initial;required,target;total,target];
elseif required>=total&&required>0,schedule=[0,initial;total,initial+(target-initial)*total/required];
else,schedule=[0,initial;max(total,eps),target];end
end
