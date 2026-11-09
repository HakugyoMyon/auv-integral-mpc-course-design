function plan=d42_task_plan(task)
% Explicit public return-set extension; keep original strict task/privacy
% validation for EVERY other field and original geometric limits.
assert(isfield(task,'homeReturnHeadingRequired')&&isequal(task.homeReturnHeadingRequired,false));
base=rmfield(task,'homeReturnHeadingRequired');plan=d11_task_plan(base);plan.task=task;
end
