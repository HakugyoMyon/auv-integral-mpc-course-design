function open_simulink_v3()
% Keep project-local dependency paths while the interactive model is open.
persistent pathScope
root=fileparts(mfilename('fullpath'));addpath(root);
if isempty(pathScope),pathScope=project_environment();end
open_system(fullfile(root,'models','auv_v3_closed_loop.slx'));
fprintf('AUV v3 model ready. Defaults: CaseIndex=1, ControllerMode=2 (MPC), stop=200s.\n');
fprintf('Use run_simulink_v3_case(caseIndex,mode) or run_simulink_v3_validation() for logged comparisons.\n');
end
