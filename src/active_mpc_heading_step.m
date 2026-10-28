function [rudder,memory,info] = active_mpc_heading_step(z,reference,memory,C,P,trim,M,low,forceFailure)
% v3 mainline uses the exact validated HOLD_I kernel; no duplicate equations.
if nargin<9,forceFailure=false;end
assert(isfield(M,'previewPolicy') && strcmp(M.previewPolicy,'hold') && ~M.ablation.preview, ...
 'AUV:ActiveVersion','Current heading step requires v3 held-reference integral MPC.');
[rudder,memory,info]=preview_candidate_heading_step(z,reference,memory,C,P,trim,M,low,0,forceFailure);
end
