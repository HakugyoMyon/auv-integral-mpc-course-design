function [config,plan,spec,driver,qualification]=sta_gui_prepare(config,mode,sceneKind,worldSeed)
% GUI boundary: geometry check does not expose obstacle geometry to planner.
assert(any(strcmp(mode,{'obstacle_free','static_unknown'})),'未知环境模式。');
assert(isscalar(worldSeed)&&isfinite(worldSeed)&&worldSeed==fix(worldSeed)&&worldSeed>=0&&worldSeed<=2147483647,'环境种子必须为0至2147483647的整数。');
geometry=auv_task_normalize(config);geometry.execution.environment='obstacle_free';geometry.execution.avoidanceEnabled=false;
[check,plan]=auv_task_validate(geometry);assert(check.valid,strjoin(check.errors,newline));
config=geometry;spec=[];driver='auv_task_run';qualification='无障碍基础任务；修改参数后仍需完整航次与独立审计。';
if strcmp(mode,'obstacle_free'),return;end
assert(all(abs(plan.task.startPose(:))<1e-12),'当前静态启动证书要求起始平面位姿[0,0,0]。');
assert(abs(plan.task.depth.transit_m-14)<1e-12,'当前静态启动自由空间证书要求航渡深度14m。');
assert(abs(plan.task.speed_m_s-.8)<1e-12&&abs(plan.task.minimumTurnRadius_m-25)<1e-12,'当前静态接口支持速度0.8m/s、最小转弯半径25m。');
assert(any(strcmp(sceneKind,{'scattered','inspection_transit','vertical_wall_wide'})),'未知障碍场景。');
if strcmp(sceneKind,'inspection_transit')
 spec=sta_scene('none',worldSeed);spec.kind=sceneKind;
 [p,~]=d16_path_preview(plan,1,0,50);
 spec.world.obstacles={struct('kind','sphere','centerNED',[p(:);14],'radius_m',1)};
 o=spec.world.obstacles{1};assert(norm(o.centerNED-spec.launchPrior.centerNED)>o.radius_m+spec.launchPrior.radius_m,'场景障碍与启动自由空间相交。');
else,spec=sta_scene(sceneKind,worldSeed,plan);end
spec.plannerOverrides=struct('maximumRouteLength_m',360,'enableHelical',true,'queryFcn',@sta_map_query_fusion);
nowUTC=posixtime(datetime('now','TimeZone','UTC'));spec.computeStopPosix=nowUTC+3*3600;spec.hardDeadlinePosix=spec.computeStopPosix;
config.execution.environment='static_unknown';config.execution.avoidanceEnabled=true;
if auv_task_is_point_task(plan.task.type),driver='sta_run';else,driver='sta_run_terrain';end
qualification='静态避障已接入执行器；感知为三维几何扫描MIL假设。任意修改的任务/种子不继承固定工况验收。';
if strcmp(plan.task.type,'survey'),qualification='区域巡测可调用静态执行器，但组合工况尚未资格化，需单独完整验证。';end
end
