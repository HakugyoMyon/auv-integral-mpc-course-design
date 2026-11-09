function auv_task_save(mission,file)
mission=auv_task_normalize(mission);[check,~]=auv_task_validate(mission);assert(check.valid,strjoin(check.errors));
assert(~isfile(file),'Mission file already exists; use a new filename.');
fid=fopen(file,'w');assert(fid>=0,'Cannot write mission file.');guard=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(mission,PrettyPrint=true));
end
