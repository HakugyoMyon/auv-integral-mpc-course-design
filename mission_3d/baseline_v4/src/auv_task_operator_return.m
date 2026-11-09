function [route,reason]=auv_task_operator_return(master,nav,time,maximumTime)
% Replan from estimated current pose/depth. No truth or reset of original IDs.
route=[];reason='operator_return_no_candidate';t=master.task;
R=auv6_rotation(nav.quaternion);
t.type='return';t.startPose=[nav.positionNED(1:2)',atan2(R(2,1),R(1,1))];
t.depth.transit_m=nav.positionNED(3);t.maximumTime_s=maximumTime-time;
if t.maximumTime_s<=10,reason='operator_return_insufficient_time';return;end
try
 route=auv_task_plan(t);
 if ~route.withinTimeBudget,route=[];reason='operator_return_time_budget';return;end
 route.operatorReturn=true;reason='operator_return_candidate_requires_fresh_MPC';
catch ex
 reason=['operator_return_geometry:',ex.message];
end
end
