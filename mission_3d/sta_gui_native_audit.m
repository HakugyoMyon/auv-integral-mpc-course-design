function result=sta_gui_native_audit(fig,referenceFile,out)
% Audit a real mouse-driven run after its UI callback has returned.
assert(~getappdata(fig,'Running'));file=getappdata(fig,'LastFile');a=load(file,'h','report','interactionLog');b=load(referenceFile,'h');
kinds=cellfun(@(x)x.kind,a.interactionLog,'UniformOutput',false);
pauses=find(strcmp(kinds,'pause_sim'));resumes=find(strcmp(kinds,'resume_sim'));stops=find(strcmp(kinds,'stop_sim'));
assert(numel(pauses)==1&&numel(resumes)==1&&numel(stops)==1);
pauseTime=a.interactionLog{pauses}.receivedAt;resumeTime=a.interactionLog{resumes}.receivedAt;
assert(pauseTime==resumeTime,'Paused simulation time advanced.');
assert(strcmp(a.report.abort,'operator_stopped_simulation')&&~a.report.physical_and_task_pass&&~a.report.return_verified);
n=numel(a.h.t);used=a.h.appliedCommandDuration_s>0;u=b.h.command(1:n,:);
delta=struct('truth',max(abs(a.h.truth-b.h.truth(1:n,:)),[],'all'),'estimate',max(abs(a.h.estimate-b.h.estimate(1:n,:)),[],'all'),'command',max(abs(a.h.command(used,:)-u(used,:)),[],'all'));
assert(delta.truth<1e-10&&delta.estimate<1e-10&&delta.command<1e-10);
audit=sta_static_audit(file,struct('physicalReplay',true,'controlReplay',true));assert(audit.passed);
dots=findobj(fig,'Tag','ObservedMap');assert(~isempty(dots)&&numel(dots.XData)>0&&numel(dots.XData)<=1500);
result=struct('passed',true,'rawFile',file,'pauseReceivedAt_s',pauseTime,'resumeReceivedAt_s',resumeTime,'stopReceivedAt_s',a.interactionLog{stops}.receivedAt,'maximumConsumedPrefixDifferences',delta,'observedMapDisplayed',true,'endedRunButtonEnabled',strcmp(findall(fig,'Tag','StaticRun').Enable,'on'),'report',a.report,'scope','Actual native mouse Run/Pause/Resume/Stop, archived command-queue replay and physics/control audit; close protection was separately observed using the real Alt-F4 action while paused. No physical hovering/stopping claim.');
if ~isfolder(out),mkdir(out);end
fid=fopen(fullfile(out,'NATIVE_GUI_CONTROLS.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
