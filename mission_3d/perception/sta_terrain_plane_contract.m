function result=sta_terrain_plane_contract()
% Affine geometry tests use truth only in sensor synthesis, never in helper.
c=auv6_altimeter_config();c.noiseStd=0;c.dropoutProbability=0;
x=zeros(18,1);x(1:3)=[4;-3;12];x(4)=1;
flat=@(n,e)24+zeros(size(n+e));s=RandStream('mt19937ar','Seed',9561);
p=auv6_altimeter(x,1,c,flat,s,false);p=rmfield(p,{'attitudeAtSample','depthAtSample'});
a=sta_terrain_plane(p,x,c,[]);assert(a.valid&&norm(a.rawGradientNE)<1e-6&&abs(a.bottomDepth_m-24)<1e-5);
slope=[.03;-.02];terrain=@(n,e)24+slope(1)*n+slope(2)*e;
p=auv6_altimeter(x,2,c,terrain,s,false);p=rmfield(p,{'attitudeAtSample','depthAtSample'});
b=sta_terrain_plane(p,x,c,[]);assert(b.valid&&norm(b.rawGradientNE-slope)<1e-5&&abs(b.bottomDepth_m-terrain(x(1),x(2)))<1e-5);
% Tilted pose about an arbitrary body axis, safely below baseline maxTilt.
axis=[1;2;-.5];axis=axis/norm(axis);angle=.15;x(4:7)=[cos(angle/2);axis*sin(angle/2)];
p=auv6_altimeter(x,3,c,terrain,s,false);p=rmfield(p,{'attitudeAtSample','depthAtSample'});
d=sta_terrain_plane(p,x,c,[]);assert(d.valid&&norm(d.rawGradientNE-slope)<1e-5);
invalid=p;invalid.valid=false;e=sta_terrain_plane(invalid,x,c,d);assert(~e.valid&&all(isnan(e.gradientNE)));
steep=@(n,e)24+.12*n;
p=auv6_altimeter(x,4,c,steep,s,false);p=rmfield(p,{'attitudeAtSample','depthAtSample'});
f=sta_terrain_plane(p,x,c,[]);assert(f.valid&&f.gradientClipped&&abs(norm(f.gradientNE)-.07)<1e-10&&abs(f.rawGradientNE(1)-.12)<1e-5);
p=auv6_altimeter(x,3.2,c,terrain,s,false);p=rmfield(p,{'attitudeAtSample','depthAtSample'});
previous=d;previous.gradientNE=[0;0];g=sta_terrain_plane(p,x,c,previous);
assert(g.valid&&norm(g.gradientNE-(.2/2.2)*g.rawGradientNE)<1e-10);
assert(g.time==p.time&&g.captureTime==p.time&&isequal(g.originNE,x(1:2))&&~isfield(g,'world')&&~isfield(g,'truth'));
result=struct('passed',true,'checks',7,'scope','Synthetic zero-noise affine flat/slope/tilted-pose, invalid, nominal clipping, smoothing and capture metadata checks. Does not establish unknown-terrain bounds or closed-loop mission success.');
end
