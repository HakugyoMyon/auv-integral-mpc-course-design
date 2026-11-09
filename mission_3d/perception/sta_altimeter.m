function packet=sta_altimeter(x,t,c,terrain,stream,forcedDropout,world)
% Baseline RNG and timing preserved; obstacle returns cannot masquerade as bottom.
packet=auv6_altimeter(x,t,c,terrain,stream,forcedDropout);
packet=rmfield(packet,{'attitudeAtSample','depthAtSample'});
[origin,hits]=sta_bottom_geometry(x,c.mount,c.beams,c.maxRange,terrain);
occluded=false(1,size(c.beams,2));
for k=1:numel(occluded)
 if all(isfinite(hits(:,k))),occluded(k)=sta_downlooking_occluded([origin';hits(:,k)'],world,0);end
end
packet.beamValid(occluded)=false;packet.ranges(occluded)=NaN;packet.valid=all(packet.beamValid);
packet.occlusionChecked=true;packet.occluded=any(occluded);
packet.occlusionScope='Conservative geometric static-obstacle invalidation; no obstacle identity or geometry. DVL bottom lock and real acoustics are not modeled.';
end
