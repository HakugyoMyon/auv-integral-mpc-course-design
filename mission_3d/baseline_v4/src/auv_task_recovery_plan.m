function [route,reason]=auv_task_recovery_plan(master,ledger,nav,time)
% New route geometry only; ledger always stays in original master IDs.
if ~strcmp(master.task.type,'visit')
 [route,reason]=d42_remaining_plan(master,ledger,nav,time);
 if ~isempty(route)
  route.task.depth.transit_m=nav.positionNED(3);route=auv_task_vertical_screen(route,nav.positionNED(3));
  if ~route.withinTimeBudget,route=[];reason='vertical_recovery_reserve_exceeds_budget';end
 end
 return;
end
route=[];reason='no_remaining_visits';ids=ledger.originalVisitOrder(ledger.counts(ledger.originalVisitOrder)<1);
if isempty(ids),return;end
t=master.task;R=auv6_rotation(nav.quaternion);t.startPose=[nav.positionNED(1:2)',atan2(R(2,1),R(1,1))];
t.depth.transit_m=nav.positionNED(3);t.maximumTime_s=master.task.maximumTime_s-time;if t.maximumTime_s<=10,reason='no_remaining_time';return;end
t.visit.pointsNED=master.targets.positionsNED(ids,:);t.visit.priority=(numel(ids):-1:1)';
route=d42_task_plan(t);route=auv_task_vertical_screen(route,nav.positionNED(3));if ~route.withinTimeBudget||~route.allTargetsGeometricallyPlanned,route=[];reason='remaining_route_time_or_geometry';return;end
reason='original_order_remaining_visits';
end
