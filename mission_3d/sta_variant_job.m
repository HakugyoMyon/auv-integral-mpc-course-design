function sta_variant_job(config,budget,out,spec,driver,maximumTime)
assert(any(strcmp(driver,{'sta_run','sta_run_height','sta_run_terrain'})));
if nargin<6,maximumTime=config.task.maximumTime_s;end
if ~isfolder(out),mkdir(out);end
try
 sta_snapshot(out);r=feval(driver,config,budget,out,maximumTime,spec);
 fid=fopen(fullfile(out,'DONE.txt'),'w');fprintf(fid,'%s\n',r.abort);fclose(fid);
catch err
 fid=fopen(fullfile(out,'ERROR.txt'),'w');fprintf(fid,'%s',getReport(err,'extended'));fclose(fid);
end
end
