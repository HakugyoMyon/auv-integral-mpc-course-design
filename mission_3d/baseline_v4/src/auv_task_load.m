function mission=auv_task_load(file)
info=dir(file);assert(isscalar(info)&&~info.isdir&&info.bytes<=2*1024^2,'Mission file missing or too large.');
mission=auv_task_normalize(jsondecode(fileread(file)));[check,~]=auv_task_validate(mission);assert(check.valid,strjoin(check.errors));
end
