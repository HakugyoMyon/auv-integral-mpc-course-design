function [config,spec,intent]=sta_case_config(name,root,queryPolicy)
% Deterministic qualification fixture builder; no MIL or random-task generation.
% Root must already have added src/planning/perception to the MATLAB path.
if nargin<2||isempty(root),root=fileparts(fileparts(mfilename('fullpath')));end
if nargin<3,queryPolicy='guard_only';end
name=char(name);queryPolicy=char(queryPolicy);
assert(any(strcmp(name,{'single','random82001','random82002','vertical_wall',...
 'inspection_transit','survey_transit','sensor_loss'})),'Unknown qualified case name.');
assert(any(strcmp(queryPolicy,{'guard_only','whole_ball_free_fusion'})));
missionName='visit';
if strcmp(name,'inspection_transit'),missionName='inspection';
elseif strcmp(name,'survey_transit'),missionName='survey';end
config=auv_task_load(fullfile(root,'baseline_v4','missions',[missionName,'.json']));
if strcmp(missionName,'visit')
 config.task.visit.pointsNED=[180,0,14];config.task.visit.priority=1;
 config.task.maximumTime_s=1500;config.execution.seed=1103;
end
config.task.id=['static_',name];
% Preserve original geometry-validation flags before enabling static sensing.
assert(strcmp(config.execution.environment,'obstacle_free')&&~config.execution.avoidanceEnabled);
[check,plan]=auv_task_validate(config);assert(check.valid,strjoin(check.errors));
switch name
 case 'single',spec=sta_scene('single',81001);
 case 'random82001',spec=sta_scene('scattered',82001,plan);
 case 'random82002',spec=sta_scene('scattered',82002,plan);
 case 'vertical_wall',spec=sta_scene('vertical_wall_wide',82003,plan);
 case {'inspection_transit','survey_transit'}
  spec=sta_scene('none',81001);
  spec.kind=name;
  [point,~]=d16_path_preview(plan,1,0,50);
  obstacle=struct('kind','sphere','centerNED',[point(:);14],'radius_m',1);
  spec.world.obstacles={obstacle};
  assert(norm(obstacle.centerNED-spec.launchPrior.centerNED)>obstacle.radius_m+spec.launchPrior.radius_m,...
   'Transit obstacle overlaps declared launch free prior.');
 case 'sensor_loss'
  spec=sta_scene('none',81001);spec.scannerOverrides=struct('dropoutProbability',1);
end
spec.caseName=name;
spec.plannerOverrides=struct('maximumRouteLength_m',360,'enableHelical',true);
if strcmp(queryPolicy,'guard_only'),spec.plannerOverrides.queryFcn=@sta_map_query;
else,spec.plannerOverrides.queryFcn=@sta_map_query_fusion;end
config.execution.environment='static_unknown';config.execution.avoidanceEnabled=true;
maximumTime=config.task.maximumTime_s;if strcmp(name,'sensor_loss'),maximumTime=30;end
intent=struct('schema','auv.static_case_intent.v1','name',name,...
 'queryPolicy',queryPolicy,'expectedPass',~strcmp(name,'sensor_loss'),...
 'maximumTime_s',maximumTime,'nominalGeometryValidated',true,...
 'nominalGeometricETA_s',check.geometricETA_s,...
 'simulationSeed',config.execution.seed,'worldSeed',spec.seed,...
 'scope','Named reproducibility fixtures only, not observed outcomes or a task randomizer. Scenario world and geometry audit belong exclusively to sensor/offline use; intent is not a planner input. A later independent run must explicitly assign its own compute wall budget; this builder preserves the engineering deadline.');
if strcmp(name,'sensor_loss')
 intent.expectation='Negative case: all static scan packets unavailable. No exact abort string is presumed. Pass maximumTime_s=30 to sta_run; retain1500s task budget so nominal geometry validation remains valid.';
else
 intent.expectation='Intended full-task positive; expectedPass is a test expectation, never a claim that this configuration has passed.';
end
end
