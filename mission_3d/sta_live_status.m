function s=sta_live_status(t,xhat,confirmed,total,operatorReturn,abort,active,map)
% Display data only: observed voxel centers, not simulator truth or guard bounds.
s=struct('time_s',t,'estimatedNED',xhat(1:3),'confirmed',confirmed,'total',total,...
 'operatorReturn',operatorReturn,'abort',abort,'avoidanceActive',active,...
 'acceptedStaticScans',map.acceptedScans,'mapGuardVoxels',size(map.occupiedCentersNED,1));
if mod(round(t*10),10)~=0,return;end
p=map.occupiedCentersNED;
ids=find(abs(p(:,1)-xhat(1))<=60&abs(p(:,2)-xhat(2))<=60&abs(p(:,3)-xhat(3))<=30);
if numel(ids)>1500,ids=ids(unique(round(linspace(1,numel(ids),1500))));end
s.observedMapNED=p(ids,:);
s.observedMapScope='Historical sensor/nav-derived voxel centers within local display bounds; deterministically sampled. Includes seabed. Not obstacle truth, fused forbidden-set boundary or a complete free-space map.';
end
