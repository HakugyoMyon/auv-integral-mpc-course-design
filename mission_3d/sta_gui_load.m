function definition=sta_gui_load(file)
d=jsondecode(fileread(file));
if isfield(d,'schema')&&strcmp(d.schema,'auv.static_gui.v1')
 definition=sta_gui_definition(d.config,d.mode,d.sceneKind,d.worldSeed);
else
 assert(isfield(d,'task')&&isfield(d,'execution'),'文件不是任务配置；不能导入离线障碍真值地图。');
 definition=sta_gui_definition(d,d.execution.environment,'scattered',82002);
end
end
