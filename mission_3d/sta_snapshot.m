function sta_snapshot(out)
root=fileparts(mfilename('fullpath'));dest=fullfile(out,'source_snapshot');
if ~isfolder(dest),mkdir(dest);end
copyfile(fullfile(root,'*.m'),dest);copyfile(fullfile(root,'planning'),fullfile(dest,'planning'));
copyfile(fullfile(root,'perception'),fullfile(dest,'perception'));copyfile(fullfile(root,'baseline_v4','src'),fullfile(dest,'baseline_src'));
copyfile(fullfile(root,'baseline_v4','input_budget.json'),fullfile(dest,'input_budget.json'));
f=dir(fullfile(dest,'**','*'));manifest=struct('relativePath',{},'sha256',{});
for k=1:numel(f)
 if f(k).isdir,continue;end
 path=fullfile(f(k).folder,f(k).name);fid=fopen(path,'rb');bytes=fread(fid,Inf,'*uint8');fclose(fid);
 md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);digest=typecast(md.digest(),'uint8');
 manifest(end+1)=struct('relativePath',erase(path,[dest,filesep]),'sha256',lower(reshape(dec2hex(digest,2)',1,[])));
end
fid=fopen(fullfile(out,'source_manifest.json'),'w');fprintf(fid,'%s',jsonencode(manifest,PrettyPrint=true));fclose(fid);
end
