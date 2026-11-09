function probe=sta_nominal_probe(route,mission,xhat,referenceDepth,targetDepth,map,c)
% Long lookahead to begin slow maneuvers early; UNKNOWN is not called free.
s=(0:1:c.threatLookahead_m)';points=zeros(numel(s)*numel(c.bodyX_m),3);xs=c.bodyX_m(:);
for k=1:numel(s)
 [p,yaw]=d16_path_preview(route,mission.segment,mission.progress,s(k));
 z=referenceDepth+min(max(targetDepth-referenceDepth,-c.maximumDepthRate_m_s*s(k)/c.maximumSpeed_m_s),c.maximumDepthRate_m_s*s(k)/c.maximumSpeed_m_s);
 ids=(k-1)*numel(xs)+(1:numel(xs));points(ids,:)=[p(:)',z]+xs*[cos(yaw),sin(yaw),0];
end
radius=sqrt(c.bodyCrossRadius_m^2+(max(diff(xs))/2)^2);
margin=radius+c.navRadius95_m+c.trackingTube_m+c.residualClearance_m+max(abs(xs))*2*sin(c.attitudeBudget_rad/2)+.5;
q=sta_map_query(map,points,margin);states=reshape(q.state,numel(xs),numel(s));
occupied=any(states==1,1);first=find(occupied,1);
distance=Inf;if ~isempty(first),distance=s(first);end
probe=struct('knownObstacleAhead',any(occupied),'firstOccupiedDistance_m',distance,...
 'occupiedSamples',sum(q.occupied),'unknownSamples',sum(q.unknown),'margin_m',margin,...
 'scope','Estimated-route occupied-threat screening only. Unknown is not free and is handled by actual committed-prefix checks; no world geometry input.');
end
