function cleanup = project_environment()
% Scoped MATLAB path setup; caller retains cleanup for the duration of work.
root=fileparts(mfilename('fullpath')); prior=path;
cleanup=onCleanup(@() path(prior));
addpath(fullfile(root,'src')); addpath(fullfile(root,'tests'));
mss=fullfile(root,'references','code','MSS');
folders={'CRAFT/AUV/models','LIBRARY','LIBRARY/modeling','LIBRARY/kinematics','GNC','HYDRO','INS/functions'};
for k=1:numel(folders), addpath(fullfile(mss,folders{k})); end
assert(strcmp(which('remus100'),fullfile(mss,'CRAFT','AUV','models','remus100.m')));
end
