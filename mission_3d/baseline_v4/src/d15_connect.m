function [segments,audit]=d15_connect(startPose,goalPose,radius,chart,role,target,minimumLength)
% Thread-compatible six-family Dubins connection; known chart only.
if nargin<5,role='transit';end;if nargin<6,target=0;end;if nargin<7,minimumLength=0;end
assert(radius>=25&&~isfield(chart,'truth')&&~isfield(chart,'dynamicObstacles'));
startPose=startPose(:);goalPose=goalPose(:);segments=struct([]);
audit=struct('accepted',false,'length_m',Inf,'toolbox_type','','candidates',0,'reason','start_or_goal_chart_clearance','implementation','pure_numeric_MIT_adaptation');
if ~d11_chart_clear([d11_segment(startPose,'line',0,Inf,0),d11_segment(goalPose,'line',0,Inf,0)],chart),return;end
paths=d15_dubins_candidates(startPose,goalPose,radius);audit.candidates=numel(paths);audit.reason='no_chart_clear_family_with_required_length';
for k=1:numel(paths)
 path=paths(k);if path.length_m<minimumLength-1e-8,continue;end
 pose=startPose;parts=struct([]);
 for j=1:numel(path.word)
  length=path.lengths(j);if length<1e-8,continue;end
  type=path.word(j);
  if type=='S',part=d11_segment(pose,'line',length,Inf,0,role,target);
  else,direction=1;if type=='R',direction=-1;end;part=d11_segment(pose,'arc',length,radius,direction,role,target);end
  if isempty(parts),parts=part;else,parts(end+1)=part;end
  pose=[part.finish;part.heading1];
 end
 assert(norm(pose(1:2)-goalPose(1:2))<1e-6&&abs(atan2(sin(pose(3)-goalPose(3)),cos(pose(3)-goalPose(3))))<1e-7,'Dubins endpoint mismatch.');
 if ~isempty(parts)&&~d11_chart_clear(parts,chart),continue;end
 segments=parts;audit.accepted=true;audit.length_m=path.length_m;audit.toolbox_type=path.word;audit.reason='geometric_candidate_only';return;
end
end
