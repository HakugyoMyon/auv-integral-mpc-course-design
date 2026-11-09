function report=run_static_case(name,outputFolder,queryPolicy,executionVariant)
% Named reproducibility entry; this is not a random task generator.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'planning'),fullfile(root,'perception'),fullfile(root,'baseline_v4','src'));
if nargin<1,name='single';end
if nargin<2||isempty(outputFolder),outputFolder=fullfile(root,'results',[char(name),'_',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))]);end
if nargin<3,queryPolicy='guard_only';end
if nargin<4,executionVariant='fixed_depth';end
switch char(executionVariant)
 case 'fixed_depth',driver='sta_run';
 case 'altitude_inheritance_v1',driver='sta_run_height';
 case 'altitude_plane_nominal_v2',driver='sta_run_terrain';
 otherwise,error('Unknown execution variant; choose fixed_depth, altitude_inheritance_v1, or altitude_plane_nominal_v2.');
end
[config,spec,intent]=sta_case_config(name,root,queryPolicy);
intent.executionVariant=char(executionVariant);intent.driver=driver;
% The preserved fixture's absolute dates described its development session.
% A new user invocation has its own explicit 3h wall budget.
spec.developmentComputeStopPosix=spec.computeStopPosix;
spec.computeStopPosix=posixtime(datetime('now','TimeZone','UTC'))+3*3600;spec.hardDeadlinePosix=spec.computeStopPosix;
assert(~isfolder(outputFolder),'Use a fresh output folder.');mkdir(outputFolder);
fid=fopen(fullfile(outputFolder,'validation_intent.json'),'w');fprintf(fid,'%s',jsonencode(intent,PrettyPrint=true));fclose(fid);
sta_snapshot(outputFolder);
report=feval(driver,config,fullfile(root,'baseline_v4','input_budget.json'),outputFolder,intent.maximumTime_s,spec);
end
