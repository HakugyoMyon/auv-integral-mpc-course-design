function [master,pointer,audit]=sta_replan_anchor(activeRoute,mission,xhat)
% Original-route forward search anchor for a fresh physical local connection.
% An anchor is NOT achieved progress, a pose change, or task confirmation.
assert(isstruct(activeRoute)&&isstruct(mission));
assert(numel(xhat)>=7&&all(isfinite(xhat(1:7)))&&abs(norm(xhat(4:7))-1)<1e-3);
local=isfield(activeRoute,'staticAvoidance')&&activeRoute.staticAvoidance;
master=activeRoute;pointer=mission;depth=0;legacy=false;
% Unwrap to the original nonlocal route; tolerate older nested detour records.
while isfield(master,'staticAvoidance')&&master.staticAvoidance
 assert(isfield(master,'resumeRoute')&&isfield(master,'resumePoint'),'Missing original-route resume anchor.');
 assert(depth<8,'Excessively nested local route chain.');
 if isfield(master,'originPointer'),seed=master.originPointer;
 else,seed=master.resumePoint;legacy=true;end
 master=master.resumeRoute;depth=depth+1;
 pointer.segment=seed.segment;pointer.progress=seed.progress;
end
assert(isfield(pointer,'segment')&&isfield(pointer,'progress'));
originalSeed=struct('segment',pointer.segment,'progress',pointer.progress);
if local
 pointer.projectionMaxDistance_m=25;pointer.projectionMaxHeadingError_rad=pi/2;
 pointer.projectionWindow_m=150;
end
[candidate,projection]=sta_project_master(master,xhat,pointer);
if projection.accepted,pointer=candidate;end
% Terminal flags belong to the active geometry, never inherit as master proof.
pointer.finished=false;pointer.complete=false;pointer.returnVerified=false;
pointer.terminalVerified=false;pointer.terminalApproachActive=false;
reason='retained_current_original_anchor';
if local,reason='retained_original_search_origin';end
if legacy,reason='legacy_future_resume_anchor';end
if projection.accepted,reason='limited_forward_estimated_projection';end
audit=struct('accepted',true,'projectionAccepted',projection.accepted,...
 'reason',reason,'unwrappedDetours',depth,'legacyFutureAnchor',legacy,'seed',originalSeed,...
 'anchor',struct('segment',pointer.segment,'progress',pointer.progress),...
 'projection',projection,...
 'scope','Estimated original-route search anchor only; planned future joins are never treated as achieved progress. Failed projection retains stored search origin, except explicitly flagged legacy routes. Requires new curvature-constrained connection, map and fresh MPC admission. Does not advance mission ledger, certify reached progress, or complete skipped targets.');
end
