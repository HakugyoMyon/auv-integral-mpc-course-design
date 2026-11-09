function cache=d8_preview_cache(p,f)
assert(all(f.normal(1,:)==0)&&max(abs(sum(f.normal.^2,1)-1))<1e-12);
cache.inverseMass=p.M\eye(6);
cache.position=f.position;cache.normal=f.normal;
cache.dragFactor=pi*f.efficiency*f.AR;
end
