function sta_gui_save(definition,file)
d=sta_gui_definition(definition.config,definition.mode,definition.sceneKind,definition.worldSeed);
fid=fopen(file,'w');assert(fid>=0,'无法写入配置文件。');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s',jsonencode(d,PrettyPrint=true));
end
