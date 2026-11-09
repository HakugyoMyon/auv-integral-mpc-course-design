function memory=d3_initialize_memory(lastApplied,m)
% Explicit driver command, not inferred from actual lagged actuator position.
assert(numel(lastApplied)==5&&all(isfinite(lastApplied))&&all(lastApplied>=m.uMin-1e-8)&&all(lastApplied<=m.uMax+1e-8));
memory=struct('previousX',zeros(15,1),'previousU',(lastApplied-m.u0)./m.scale,'disturbance',zeros(6,1),...
 'warmStart',zeros(5*m.Nc+1,1),'hasPrevious',false,'consecutiveFailures',0);
end
