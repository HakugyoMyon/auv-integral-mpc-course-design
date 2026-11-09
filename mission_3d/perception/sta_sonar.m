function packet=sta_sonar(world,xTruth,time,c,stream)
% Only this simulator reads world/truth. No world or object identity in packet.
% Conservative angular-cell return: inflate geometry by the furthest cell-ray
% separation at max range, then raycast. Thus free prefixes do not interpolate
% unobserved gaps between zero-width rays. This is NOT an acoustic image model.
assert(isstruct(world)&&isscalar(world));
if isfield(world,'obstacles'),objects=world.obstacles;else,objects=struct([]);end
q=xTruth(4:7);R=auv6_rotation(q);origin=xTruth(1:3)+R*c.mountFRD;
[az,el]=meshgrid(c.azimuth_rad,c.elevation_rad);
local=[cos(el(:)).*cos(az(:)),cos(el(:)).*sin(az(:)),sin(el(:))]';directions=R*local;
half=sqrt((max(diff(c.azimuth_rad))/2)^2+(max(diff(c.elevation_rad))/2)^2);
guard=2*c.maxRange_m*sin(half/2);count=numel(az);nearest=inf(1,count);
for k=1:numel(objects)
 if iscell(objects),o=objects{k};else,o=objects(k);end
 if isfield(o,'radius_m'),radius=o.radius_m;else,radius=o.radius;end
 assert(isfinite(radius)&&radius>0);
 if strcmp(o.kind,'sphere')
  if isfield(o,'centerNED'),center=o.centerNED(:);else,center=o.center(:);end
  v=origin-center;b=sum(directions.*v,1);disc=b.^2-(dot(v,v)-(radius+guard)^2);
  hit=inf(1,count);valid=disc>=0;near=-b(valid)-sqrt(disc(valid));far=-b(valid)+sqrt(disc(valid));
  near(near<0&far>=0)=0;near(far<0)=Inf;hit(valid)=near;
 elseif strcmp(o.kind,'cylinder')
  center=o.centerNE(:);if isfield(o,'depthLimits_m'),depth=o.depthLimits_m;else,depth=[o.top,o.bottom];end
  rr=radius+guard;top=depth(1)-guard;bottom=depth(2)+guard;
  xy=origin(1:2)-center;a=sum(directions(1:2,:).^2,1);b=2*sum(directions(1:2,:).*xy,1);d=dot(xy,xy)-rr^2;
  discriminant=b.^2-4*a*d;hit=inf(1,count);valid=discriminant>=0&a>1e-12;
  for sign=[-1,1]
   root=inf(1,count);root(valid)=(-b(valid)+sign*sqrt(discriminant(valid)))./(2*a(valid));
   z=origin(3)+root.*directions(3,:);good=root>=0&z>=top&z<=bottom;hit(good)=min(hit(good),root(good));
  end
  for z=[top,bottom]
   distance=(z-origin(3))./directions(3,:);xyhit=origin(1:2)+directions(1:2,:).*distance;
   good=abs(directions(3,:))>1e-12&distance>=0&sum((xyhit-center).^2,1)<=rr^2;
   hit(good)=min(hit(good),distance(good));
  end
  if norm(xy)<=rr&&origin(3)>=top&&origin(3)<=bottom,hit(:)=0;end
 else,error('Unsupported static obstacle geometry; do not silently mark it free.');end
 nearest=min(nearest,hit);
end
% Clip each beam at first water-column boundary; unobserved seabed is never
% penetrated by a free certificate. Terrain appears only inside the simulator.
range=(0:c.terrainStep_m:c.maxRange_m)';N=origin(1)+range*directions(1,:);E=origin(2)+range*directions(2,:);Z=origin(3)+range*directions(3,:);
if isfield(world,'terrainFcn'),bottom=world.terrainFcn(N,E);
elseif isfield(world,'bottomDepth_m'),bottom=world.bottomDepth_m+zeros(size(N));
else,error('Sensor world must declare terrainFcn or bottomDepth_m.');end
surface=0;if isfield(world,'surfaceDepth_m'),surface=world.surfaceDepth_m;end
padding=(guard+c.terrainStep_m/2)*sqrt(1+c.terrainGradientBound^2);
unsafe=Z<=surface+padding|bottom-Z<=padding;[has,index]=max(unsafe,[],1);
water=inf(1,count);water(logical(has))=max(0,range(index(logical(has)))-c.terrainStep_m);
nearest=min(nearest,water);hit=nearest<=c.maxRange_m;
prefix=min(nearest,c.maxRange_m);noise=(2*rand(stream,1,count)-1)*c.rangeErrorBound_m;
measured=prefix+noise;state=2*ones(1,count,'uint8');state(hit)=1;
state(prefix<c.minRange_m)=0;
scanValid=rand(stream)>=c.dropoutProbability;
if ~scanValid,state(:)=0;end
packet=struct('schema','auv.static_sonar_packet.v1','time',time,'availableTime',time+c.latency_s,...
 'azimuth_rad',c.azimuth_rad,'elevation_rad',c.elevation_rad,'mountFRD',c.mountFRD,...
 'minRange_m',c.minRange_m,'maxRange_m',c.maxRange_m,'rangeErrorBound_m',c.rangeErrorBound_m,...
 'range_m',reshape(measured,size(az)),'rayState',reshape(state,size(az)),...
 'scanValid',scanValid,'angularCellGuard_m',guard,...
 'scope','Geometric conservative finite angular-cell return envelope with bounded noise; not acoustic imagery or proof of installed3D hardware.');
end
