function distance=sta_world_clearance(x,world)
% OFFLINE observer only. No controller/planner may call with true world.
R=auv6_rotation(x(4:7));distance=Inf;
for k=1:numel(world.obstacles)
 if iscell(world.obstacles),o=world.obstacles{k};else,o=world.obstacles(k);end
 if strcmp(o.kind,'sphere')
  point=R'*(o.centerNED(:)-x(1:3));nearest=min(max(point,[-1.30;-.205;-.205]),[.65;.205;.205]);
  d=norm(point-nearest)-o.radius_m;
 elseif strcmp(o.kind,'cylinder')
  xs=linspace(-1.30,.65,7);points=x(1:3)+R*[xs;zeros(2,numel(xs))];radius=sqrt((sqrt(2)*.205)^2+(max(diff(xs))/2)^2);
  radial=max(0,vecnorm(points(1:2,:)-o.centerNE(:),2,1)-o.radius_m);
  vertical=max([o.depthLimits_m(1)-points(3,:);points(3,:)-o.depthLimits_m(2);zeros(1,numel(xs))],[],1);
  d=min(hypot(radial,vertical))-radius;
 else,error('Offline clearance has unsupported obstacle kind.');end
 distance=min(distance,d);
end
end
