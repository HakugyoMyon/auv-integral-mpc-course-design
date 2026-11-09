function report=run_static_mission(taskOrConfig,outputFolder,sceneKind,worldSeed,maximumTime,plannerOptions)
%RUN_STATIC_MISSION Isolated unknown-static-obstacle MIL entry point.
% This is a candidate extension; consult QUALIFICATION_PLAN.md before making claims.
% World geometry is not a planner input. New multi-elevation sonar assumption.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'baseline_v4','src'),fullfile(root,'planning'),fullfile(root,'perception'));
if nargin<1||isempty(taskOrConfig)
 config=auv_task_load(fullfile(root,'baseline_v4','missions','visit.json'));
 config.task.id='static_single_visit';config.task.visit.pointsNED=[180,0,14];
 config.task.visit.priority=1;config.task.maximumTime_s=1500;
elseif isstruct(taskOrConfig),config=taskOrConfig;
else,config=sta_load(taskOrConfig);end
if nargin<2||isempty(outputFolder),outputFolder=fullfile(root,'results',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));end
if nargin<3||isempty(sceneKind),sceneKind='single';end
if nargin<4,worldSeed=81001;end
if nargin<5||isempty(maximumTime),maximumTime=config.task.maximumTime_s;end
if nargin<6,plannerOptions=struct();end
assert(isstruct(plannerOptions)&&all(ismember(fieldnames(plannerOptions),fieldnames(sta_planner_config()))));
config.execution.environment='static_unknown';config.execution.avoidanceEnabled=true;
g=config;g.execution.environment='obstacle_free';g.execution.avoidanceEnabled=false;
[valid,nominal]=auv_task_validate(g);assert(valid.valid,strjoin(valid.errors));
spec=sta_scene(sceneKind,worldSeed,nominal);
spec.plannerOverrides=plannerOptions;
% Development deadline is set by the owner during this bounded engineering
% session. A later standalone user run has its own explicit 3h wall budget.
wallNow=posixtime(datetime('now','TimeZone','UTC'));
spec.computeStopPosix=wallNow+3*3600;spec.hardDeadlinePosix=spec.computeStopPosix;
report=sta_run(config,fullfile(root,'baseline_v4','input_budget.json'),outputFolder,maximumTime,spec);
end

