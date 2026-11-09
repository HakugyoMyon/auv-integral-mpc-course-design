function packet=sta_payload_sensor(x,t,c,terrain,stream,forcedDropout,world)
% Entire abstract continuous imaging volume, not merely five sampled rays.
packet=d11_payload_sensor(x,t,c,terrain,stream,forcedDropout);
[origin,hits]=sta_bottom_geometry(x,c.mount,c.rays,c.maximumRange_m,terrain);
occluded=false;
if all(isfinite(hits),'all')
 % Include center and corners. Inflate for permitted nonplanar bottom patch;
 % this can reject visible footprints, but cannot mark a sampled blocker free.
 occluded=sta_downlooking_occluded([origin';hits'],world,2*c.maximumPlaneDeviation_m);
end
if occluded,packet.valid=false;end
packet.occlusionChecked=true;packet.occluded=occluded;
packet.occlusionScope='Conservative full convex imaging-volume static-obstacle invalidation with bottom-plane-deviation guard. Not real acoustic/image formation; DVL bottom lock remains abstract.';
end
