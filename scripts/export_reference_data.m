function export_reference_data(sourceRoot)
% Maintainer tool: export numeric fields from the original completed study.
% No file paths, function handles, toolbox objects or personal metadata are copied.
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'reference_data','v3');
files=dir(fullfile(sourceRoot,'releases','v3_matlab_frozen','results','final_comparison_v3','*.mat'));
count=0;
for k=1:numel(files)
 if ~endsWith(files(k).name,{'_PID.mat','_MPC_v3.mat'}),continue;end
 a=load(fullfile(files(k).folder,files(k).name),'run');r=a.run;
 reference=struct('t',r.t,'z',r.z,'command',r.command,'reference',r.reference,'ey',r.ey,'C',r.C);
 save(fullfile(out,files(k).name),'reference','-v7');
 check=load(fullfile(out,files(k).name),'reference');assert(isequaln(check.reference,reference));count=count+1;
end
assert(count==24);fprintf('Exported 24 numeric-only reference files without altering arrays.\n');
end
