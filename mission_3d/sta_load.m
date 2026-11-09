function config=sta_load(file)
% Static-stage task JSON roundtrip; validates geometry without rewriting flags.
i=dir(file);assert(isscalar(i)&&~i.isdir&&i.bytes<=2*1024^2,'Invalid task file.');
config=auv_task_normalize(jsondecode(fileread(file)));
assert(any(strcmp(config.execution.environment,{'obstacle_free','static_unknown'})));
g=config;g.execution.environment='obstacle_free';g.execution.avoidanceEnabled=false;
[c,~]=auv_task_validate(g);assert(c.valid,strjoin(c.errors));
end
