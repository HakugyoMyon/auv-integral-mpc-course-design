function [ey,gamma,point] = path_geometry(position,kind)
% NED horizontal path; signed error along the left normal of its tangent.
arguments
 position (2,1) double {mustBeFinite}
 kind (1,:) char
end
switch kind
 case 'LINE'
  ey=position(2); gamma=0; point=[position(1);0];
 case 'CIRCLE'
  center=[0;50]; radius=50; d=position-center; distance=norm(d);
  assert(distance>=5,'AUV:Projection','Circle projection too close to center.');
  point=center+radius*d/distance;
  tangent=[-d(2);d(1)]/distance;
  gamma=atan2(tangent(2),tangent(1)); ey=radius-distance;
 otherwise
  error('AUV:Path','Unknown path kind.');
end
end
