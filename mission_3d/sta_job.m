function sta_job(cfg,budget,out,spec)
% Owned bounded run callback with persistent diagnostic completion markers.
if ~isfolder(out),mkdir(out);end
try
 r=sta_run(cfg,budget,out,cfg.task.maximumTime_s,spec);
 fid=fopen(fullfile(out,'DONE.txt'),'w');fprintf(fid,'%s\n',r.abort);fclose(fid);
catch err
 fid=fopen(fullfile(out,'ERROR.txt'),'w');fprintf(fid,'%s',getReport(err,'extended'));fclose(fid);
end
end
