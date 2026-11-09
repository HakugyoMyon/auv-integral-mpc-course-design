function [inherit,bounds]=sta_altitude_inheritance(master,pointer,kind,c)
% altitude_inheritance_v1: latch task phase at candidate creation, never from ledger.complete.
inherit=false;bounds=c.depthBounds_m;
assert(numel(bounds)==2&&all(isfinite(bounds))&&bounds(1)<=bounds(2));
if ~any(strcmp(kind,{'horizontal','nominal'})),return;end
if isfield(master,'operatorReturn')&&master.operatorReturn,return;end
if auv_task_is_point_task(master.task.type)||~strcmp(master.task.depth.workMode,'altitude'),return;end
if isfield(master,'staticAvoidance')&&master.staticAvoidance
 error('sta:OriginalMasterRequired','Inheritance must be decided against original nonlocal master and origin pointer.');
end
assert(pointer.segment>=1&&pointer.segment<=numel(master.segments));
if any(strcmp(master.segments(pointer.segment).role,{'departure','return'})),return;end
assert(isfield(master.task.depth,'workDepthBounds_m'),'Altitude work requires declared depth envelope.');
work=master.task.depth.workDepthBounds_m;
bounds=[max(bounds(1),work(1)),min(bounds(2),work(2))];
assert(bounds(1)<=bounds(2),'Empty task/planner altitude depth intersection.');
inherit=true;
end
