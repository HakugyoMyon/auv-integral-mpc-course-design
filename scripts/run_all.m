function run_all(includeSimulink)
% Full study; expect several minutes. Use run_design_comparison([1 2 11]) for a subset.
if nargin<1,includeSimulink=true;end
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);scope=project_environment(); %#ok<NASGU>
test_public_reproduction(false);
run_design_comparison(1:12,true);
if includeSimulink,run_simulink_v3_validation();end
for index=[1 2 11],run_sensor_study_v3(index);end
summarize_sensor_study_v3();
end
