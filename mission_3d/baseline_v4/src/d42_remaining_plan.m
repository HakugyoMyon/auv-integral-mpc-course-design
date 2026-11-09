function [route,reason]=d42_remaining_plan(master,ledger,nav,now)
% Remaining original target IDs stay in the original ledger; this generates
% route geometry only, without declaring unobserved targets complete.
route=[];reason='no_remaining_targets';remaining=find(ledger.counts<ledger.minimumObservations);
if isempty(remaining),return;end
task=master.task;R=auv6_rotation(nav.quaternion);task.startPose=[nav.positionNED(1:2)',atan2(R(2,1),R(1,1))];
deadline=master.task.maximumTime_s;task.maximumTime_s=deadline-now;
if task.maximumTime_s<=10,reason='no_remaining_time';return;end
task.id=[task.id,'_remaining'];
if strcmp(task.type,'survey')
 points=master.targets.centersNE(remaining,:);angle=task.survey.bearing_rad;
 forward=[cos(angle);sin(angle)];right=[-sin(angle);cos(angle)];
 along=points*forward;across=points*right;half=master.targets.cellSize_m/2;
 task.survey.centerNE=(forward*((min(along)+max(along))/2)+right*((min(across)+max(across))/2))';
 task.survey.length_m=max(master.targets.cellSize_m,max(along)-min(along)+2*half);
 task.survey.width_m=max(master.targets.cellSize_m,max(across)-min(across)+2*half);
else
 points=master.targets.positionsNE(remaining,:);
 if size(points,1)==1
  tangent=master.task.inspection.polylineNE(end,:)-master.task.inspection.polylineNE(1,:);tangent=tangent/norm(tangent);
  points=[points-tangent;points+tangent];
 end
 task.inspection.polylineNE=points;
end
candidate=d42_task_plan(task);
if ~candidate.allTargetsGeometricallyPlanned||candidate.geometricETA_s+10>task.maximumTime_s
 reason='insufficient_time_or_no_geometric_remaining_route';return;
end
candidate.absoluteDeadline_s=deadline;route=candidate;reason='remaining_route_candidate_only';
end
