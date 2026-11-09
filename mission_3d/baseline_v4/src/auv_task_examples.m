function missions=auv_task_examples()
% Public task inputs only. UI and scripts must share this representation.
tasks=d11_task_examples();missions=cell(size(tasks));
for k=1:numel(tasks)
 task=tasks{k};task.homeReturnHeadingRequired=false;task.maximumTime_s=1800;
 if strcmp(task.depth.workMode,'altitude'),task.depth.workDepthBounds_m=[4,24];end
 missions{k}=struct('schema','auv.mission.v1','task',task,...
  'execution',struct('environment','obstacle_free','seed',1100+k,'avoidanceEnabled',false));
end
end
