function fig=start_auv_workbench()
% One MATLAB session; no installer, compiler, MCP, CAD or external process.
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'planning'),fullfile(root,'perception'),fullfile(root,'baseline_v4','src'));
fig=sta_task_console(fullfile(root,'baseline_v4','input_budget.json'),fullfile(root,'results'));
end
