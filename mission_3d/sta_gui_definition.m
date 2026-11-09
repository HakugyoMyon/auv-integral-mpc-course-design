function definition=sta_gui_definition(config,mode,sceneKind,worldSeed)
% Serializable operator configuration only; no simulator obstacle geometry.
[config,~,~,~]=sta_gui_prepare(config,mode,sceneKind,worldSeed);
definition=struct('schema','auv.static_gui.v1','config',config,'mode',char(mode),'sceneKind',char(sceneKind),'worldSeed',worldSeed);
end
