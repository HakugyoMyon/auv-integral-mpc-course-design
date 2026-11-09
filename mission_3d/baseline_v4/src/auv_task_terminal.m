function [goal,kind]=auv_task_terminal(plan)
if isfield(plan,'terminalGoalNED'),goal=plan.terminalGoalNED(:);kind=plan.terminalKind;
else,goal=[plan.task.homePose(1:2)';plan.task.depth.return_m];kind='home';end
end
