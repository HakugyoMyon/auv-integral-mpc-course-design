function missions=auv_task_catalog()
missions=auv_task_examples();base=missions{3};base.task=rmfield(base.task,'visit');
a=base;a.task.type='transit';a.task.id='transit_to_worksite';a.task.transit=struct('destinationNED',[100,0,15],'captureRadius_m',3,'maximumBoundDistance_m',5,'depthTolerance_m',.5);a.execution.seed=1104;
b=base;b.task.type='return';b.task.id='return_to_relative_home';b.task.homePose=[-40,20,0];b.execution.seed=1105;
missions=[missions,{a,b}];
end
