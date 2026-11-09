function result=auv_task_audit_fault_windows(file)
% Verify requested outage was physically represented in received raw packets.
s=load(file,'config','payloadLog');assert(isfield(s.config.execution,'payloadUnavailableWindows_s'));
inside=0;outside=0;
for k=1:numel(s.payloadLog)
 p=s.payloadLog{k}.packet;expected=auv_task_window_active(s.config.execution,'payloadUnavailableWindows_s',p.time);
 assert(isfield(p,'forcedUnavailable')&&isequal(p.forcedUnavailable,expected),'Requested payload fault not represented in raw packet.');
 if expected,inside=inside+1;assert(~p.valid,'Packet inside forced outage was marked valid.');else,outside=outside+1;end
end
assert(inside>0&&outside>0,'Trial must contain both outage and restored observations.');
result=struct('passed',true,'unavailablePackets',inside,'outsideWindowPackets',outside,...
 'scope','Explicit MIL sensor validity loss, not hardware fault reproduction. Normal sensor generation precedes override to preserve its RNG stream.');
fid=fopen(fullfile(fileparts(file),'fault_window_audit.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
