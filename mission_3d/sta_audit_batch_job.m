function sta_audit_batch_job(files,statusFile,computeStopPosix)
if nargin<3,computeStopPosix=Inf;end
for k=1:numel(files)
 if posixtime(datetime('now','TimeZone','UTC'))>=computeStopPosix,break;end
 fid=fopen(statusFile,'w');fprintf(fid,'audit %d/%d %s',k,numel(files),files{k});fclose(fid);
 sta_audit_job(files{k});
end
fid=fopen([statusFile,'.DONE'],'w');fprintf(fid,'Audit batch ended; inspect individual flags and errors.');fclose(fid);
end

