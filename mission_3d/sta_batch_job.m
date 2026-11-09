function sta_batch_job(cases,statusFile)
% One shared MATLAB, sequential physical cases; never extends owner deadline.
for k=1:numel(cases)
 a=cases{k};now=posixtime(datetime('now','TimeZone','UTC'));
 if now>=a.spec.computeStopPosix||isfile(fullfile(fileparts(statusFile),'STOP_BATCH_REQUEST.txt')),break;end
 fid=fopen(statusFile,'w');fprintf(fid,'case=%d/%d out=%s',k,numel(cases),a.out);fclose(fid);
 if ~isfolder(a.out),mkdir(a.out);end
 try
  sta_snapshot(a.out);sta_job(a.config,a.budget,a.out,a.spec);
 catch err
  fid=fopen(fullfile(a.out,'ERROR.txt'),'w');fprintf(fid,'%s',getReport(err,'extended'));fclose(fid);
 end
end
fid=fopen([statusFile,'.DONE'],'w');fprintf(fid,'Batch ended; per-case reports determine success.');fclose(fid);
end
