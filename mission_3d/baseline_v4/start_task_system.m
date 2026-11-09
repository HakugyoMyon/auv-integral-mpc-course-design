function fig=start_task_system()
% Starts only the owned console in the current MATLAB, not another instance.
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'src'));
fig=auv_task_console(fullfile(root,'input_budget.json'),fullfile(root,'results'));
end
