function [segments,audit]=d11_connect(startPose,goalPose,radius,chart,role,target,minimumLength)
% MathWorks forward-only Dubins geometry. L increases mathematical yaw:
% with coordinates [North East], this is physical starboard/right, not port.
if nargin<5,role='transit';end
if nargin<6,target=0;end
if nargin<7,minimumLength=0;end
startPose=startPose(:);goalPose=goalPose(:);
assert(radius>=25);assert(~isfield(chart,'truth')&&~isfield(chart,'dynamicObstacles'));
segments=struct([]);audit=struct('accepted',false,'length_m',Inf,'toolbox_type','','candidates',0,'reason','start_or_goal_chart_clearance');
admission=[d11_segment(startPose,'line',0,Inf,0),d11_segment(goalPose,'line',0,Inf,0)];
if ~d11_chart_clear(admission,chart),return;end
connection=dubinsConnection('MinTurningRadius',radius);
[paths,costs]=connect(connection,startPose(:)',goalPose(:)','PathSegments','all');
[~,order]=sort(costs);audit.candidates=numel(paths);audit.reason='no_chart_clear_family_with_required_length';
for candidate=order(:)'
 if ~isfinite(costs(candidate))||costs(candidate)<minimumLength-1e-8,continue;end
 path=paths{candidate};pose=startPose(:);parts=struct([]);
 for j=1:numel(path.MotionLengths)
  length=path.MotionLengths(j);if length<1e-8,continue;end
  type=char(path.MotionTypes{j});
  if strcmp(type,'S'),part=d11_segment(pose,'line',length,Inf,0,role,target);
  else,assert(any(strcmp(type,{'L','R'})));direction=1;if strcmp(type,'R'),direction=-1;end;part=d11_segment(pose,'arc',length,radius,direction,role,target);end
  if isempty(parts),parts=part;else,parts(end+1)=part;end
  pose=[part.finish;part.heading1];
 end
 assert(norm(pose(1:2)-goalPose(1:2))<1e-6);
 assert(abs(atan2(sin(pose(3)-goalPose(3)),cos(pose(3)-goalPose(3))))<1e-7);
 if ~d11_chart_clear(parts,chart),continue;end
 segments=parts;audit.accepted=true;audit.length_m=sum([parts.length]);audit.toolbox_type=strjoin(path.MotionTypes,'');audit.reason='geometric_candidate_only';return;
end
end
