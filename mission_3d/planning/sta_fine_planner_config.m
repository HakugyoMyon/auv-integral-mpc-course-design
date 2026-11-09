function fine=sta_fine_planner_config(coarse)
% One bounded catalog refinement; leaves EVERY physical/safety budget intact.
% Evaluate against an archived PUBLIC map/xhat failure snapshot before MIL.
% Deliberately do not enable helices or change the length/depth/chart limits.
if nargin<1,coarse=sta_planner_config();end
fine=coarse;fine.turnAngles_deg=[25,30,35,40,45];
fine.sideHoldCandidates_m=[20,40,60,80,100];
% Keep the old single-hold interface in the bounded set, rather than silently
% adding an arbitrary user value and turning refinement into an open sweep.
assert(ismember(fine.sideHold_m,fine.sideHoldCandidates_m),...
 'Fine one-pass library requires the legacy hold to be one of20,40,60,80,100m.');
fine.catalogResolution='fine_once_5_angles_5_holds';
end
