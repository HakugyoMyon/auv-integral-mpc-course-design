function report=run_mission(taskOrConfig,outputFolder,maximumTime)
%RUN_MISSION Run a supported basic mission in the current MATLAB.
% report=run_mission('visit') loads missions/visit.json.
% A mission struct or a JSON filename may also be supplied.
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'src'));
if nargin<1,taskOrConfig='visit';end
if isstruct(taskOrConfig),config=taskOrConfig;
elseif any(strcmp(taskOrConfig,{'survey','inspection','visit','transit','return'}))
 config=auv_task_load(fullfile(root,'missions',[char(taskOrConfig),'.json']));
else,config=auv_task_load(taskOrConfig);end
if nargin<2,outputFolder=fullfile(root,'results',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));end
if nargin<3,maximumTime=config.task.maximumTime_s;end
report=auv_task_run(config,fullfile(root,'input_budget.json'),outputFolder,maximumTime);
end
