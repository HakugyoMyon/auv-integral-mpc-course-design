function blocked=sta_downlooking_occluded(vertices,world,inflation)
% Conservative convex viewing-volume overlap. Sensor-private world access.
% Only a separating face may prove no occlusion; uncertain overlap is invalid.
if nargin<3,inflation=0;end
blocked=false;if ~isfield(world,'obstacles')||isempty(world.obstacles),return;end
assert(size(vertices,2)==3&&all(isfinite(vertices),'all'));
% Exact finite zero-width geometric beams; payload uses the volume branch.
if size(vertices,1)==2
 for k=1:numel(world.obstacles)
  if iscell(world.obstacles),o=world.obstacles{k};else,o=world.obstacles(k);end
  if segmentHit(vertices(1,:)',vertices(2,:)',o),blocked=true;return;end
 end
 return;
end
faces=convhulln(vertices);middle=mean(vertices,1);normals=zeros(size(faces,1),3);limits=zeros(size(faces,1),1);
for k=1:size(faces,1)
 p=vertices(faces(k,:),:);n=cross(p(2,:)-p(1,:),p(3,:)-p(1,:));n=n/norm(n);limit=dot(n,p(1,:));
 if dot(n,middle)>limit,n=-n;limit=-limit;end
 normals(k,:)=n;limits(k)=limit+inflation;
end
for k=1:numel(world.obstacles)
 if iscell(world.obstacles),o=world.obstacles{k};else,o=world.obstacles(k);end
 if strcmp(o.kind,'sphere')
  center=o.centerNED(:);support=o.radius_m*ones(size(limits));
 elseif strcmp(o.kind,'cylinder')
  center=[o.centerNE(:);mean(o.depthLimits_m)];height=diff(o.depthLimits_m)/2;
  support=o.radius_m*vecnorm(normals(:,1:2),2,2)+height*abs(normals(:,3));
 else,error('Unsupported downlooking occlusion geometry.');end
 separated=any(normals*center-support>limits+1e-9);
 if ~separated,blocked=true;return;end
end
end

function hit=segmentHit(a,b,o)
v=b-a;
if strcmp(o.kind,'sphere')
 t=max(0,min(1,dot(o.centerNED(:)-a,v)/max(dot(v,v),eps)));
 hit=norm(a+t*v-o.centerNED(:))<=o.radius_m+1e-9;
elseif strcmp(o.kind,'cylinder')
 q=a(1:2)-o.centerNE(:);z=o.depthLimits_m(:)';
 hit=norm(q)<=o.radius_m&&a(3)>=z(1)&&a(3)<=z(2);if hit,return;end
 A=dot(v(1:2),v(1:2));B=2*dot(q,v(1:2));C=dot(q,q)-o.radius_m^2;
 if A>1e-14&&B^2-4*A*C>=0
  roots=(-B+[-1,1]*sqrt(B^2-4*A*C))/(2*A);
  depth=a(3)+roots*v(3);hit=any(roots>=0&roots<=1&depth>=z(1)&depth<=z(2));
 end
 if ~hit&&abs(v(3))>1e-14
  roots=(z-a(3))/v(3);points=a(1:2)+v(1:2)*roots;
  hit=any(roots>=0&roots<=1&vecnorm(points-o.centerNE(:),2,1)<=o.radius_m+1e-9);
 end
else,error('Unsupported downlooking occlusion geometry.');end
end
