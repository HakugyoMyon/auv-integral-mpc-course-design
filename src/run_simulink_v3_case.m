function result = run_simulink_v3_case(index,mode,duration)
% Compare sampled Simulink output directly to the frozen MATLAB record.
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);scope=project_environment(); %#ok<NASGU>
model='auv_v3_closed_loop';modelFile=fullfile(root,'models',[model '.slx']);
if ~bdIsLoaded(model),open_system(modelFile);end
ctx=auv_sl_configuration(index);if nargin<3,duration=ctx.scenario.duration;end
assert(isfinite(duration) && duration>0 && duration<=ctx.scenario.duration);
in=Simulink.SimulationInput(model);
in=in.setVariable('CaseIndex',index,'Workspace',model);
in=in.setVariable('ControllerMode',mode,'Workspace',model);
in=in.setModelParameter('StopTime',num2str(duration,17),'SimulationMode','normal', ...
 'SolverType','Fixed-step','Solver','ode4','FixedStep','0.02','ReturnWorkspaceOutputs','on');
out=sim(in);
t=out.slState.Time;z=matrix(out.slState);command=matrix(out.slCommand);
guidance=matrix(out.slGuidance);diagnostics=matrix(out.slSolver);
assert(max(abs(t-out.slCommand.Time))<1e-8 && max(abs(t-out.slGuidance.Time))<1e-8);
label='PID';if mode==2,label='MPC_v3';end
referenceFile=fullfile(root,'reference_data','v3',[ctx.scenario.id '_' label '.mat']);
a=load(referenceFile,'reference');reference=a.reference;ii=round(t/ctx.config.C.Ts)+1;
timeError=max(abs(reference.t(ii)-t));stateByColumn=max(abs(z-reference.z(ii,:)),[],1);
commandByColumn=max(abs(command-reference.command(ii,:)),[],1);eyError=max(abs(guidance(:,2)-reference.ey(ii)));
referenceError=max(abs(angle(exp(1i*(guidance(:,1)-reference.reference(ii,1))))));
firstState=find(any(abs(z-reference.z(ii,:))>1e-5,2),1);
firstCommand=find(any(abs(command-reference.command(ii,:))>1e-6,2),1);
firstStateTime=NaN;firstCommandTime=NaN;
if ~isempty(firstState),firstStateTime=t(firstState);end
if ~isempty(firstCommand),firstCommandTime=t(firstCommand);end
post=t>=30;steady=t>=max(0,t(end)-50);
rmse=NaN;if any(post),rmse=sqrt(mean(guidance(post,2).^2));end
rate=abs(diff(command)/ctx.config.C.Ts);actualRate=abs(diff(z(:,13:15))/ctx.config.C.Ts);
finite=all(isfinite(z),'all') && all(isfinite(command),'all');
inputViolation=max([0;reshape(command-ctx.P.upper',[],1);reshape(ctx.P.lower'-command,[],1)]);
rateViolation=max([0;reshape(rate-ctx.P.rate',[],1)]);
finalFailures=0;if mode==2,finalFailures=nnz(diagnostics(:,5));end
summary=struct('caseId',ctx.scenario.id,'method',label,'duration',duration,'actualDuration',t(end), ...
 'samples',numel(t),'timeMaxError_s',timeError,'maxStateError',max(stateByColumn), ...
 'stateErrorByColumn',stateByColumn,'maxCommandError',max(commandByColumn), ...
 'commandErrorByColumn',commandByColumn,'maxEyError_m',eyError,'maxReferenceError_rad',referenceError, ...
 'firstStateMismatchTime',firstStateTime,'firstCommandMismatchTime',firstCommandTime, ...
 'rmsePost30',rmse,'rudderTVrad',sum(abs(diff(z(:,13)))),'steadyRmse',sqrt(mean(guidance(steady,2).^2)), ...
 'finite',finite,'maxInputViolation',inputViolation,'maxRateViolation',rateViolation, ...
 'maxActualRate',max(actualRate,[],1),'finalFailures',finalFailures,'passed',false);
summary.passed=finite && abs(t(end)-duration)<1e-8 && timeError<=1e-8 && max(stateByColumn)<=1e-5 ...
 && max(commandByColumn)<=1e-6 && eyError<=1e-5 && inputViolation<=1e-8 && rateViolation<=1e-8 && finalFailures==0;
result=struct('summary',summary,'t',t,'z',z,'command',command,'guidance',guidance, ...
 'solverDiagnostics',diagnostics,'referenceFile',referenceFile,'modelFile',modelFile,'configuration',ctx.config);
outdir=fullfile(root,'results','simulink_v3');if ~isfolder(outdir),mkdir(outdir);end
tag=[ctx.scenario.id '_' label '_' num2str(duration) 's'];
save(fullfile(outdir,[tag '.mat']),'result','-v7.3');
fid=fopen(fullfile(outdir,[tag '.json']),'w');c=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));
fprintf('SIMULINK %s %s %.1fs: state %.4g command %.4g ey %.4g firstState %.4g firstCommand %.4g PASS=%d\n', ...
 ctx.scenario.id,label,duration,max(stateByColumn),max(commandByColumn),eyError,firstStateTime,firstCommandTime,summary.passed);
end
function x=matrix(series)
x=series.Data;n=numel(series.Time);
if size(x,1)~=n,x=permute(x,[ndims(x),1:ndims(x)-1]);end
x=reshape(x,n,[]);
end
