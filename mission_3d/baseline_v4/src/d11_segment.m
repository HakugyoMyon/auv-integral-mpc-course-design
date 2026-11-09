function segment=d11_segment(pose,kind,length,radius,direction,role,target)
% Coordinates are [North East yaw]. Positive yaw is North toward East.
if nargin<6,role='transit';end
if nargin<7,target=0;end
assert(numel(pose)==3&&all(isfinite(pose))&&length>=0);
pose=pose(:);center=[NaN;NaN];
if strcmp(kind,'line')
 finish=pose(1:2)+length*[cos(pose(3));sin(pose(3))];heading1=pose(3);radius=Inf;direction=0;
else
 assert(strcmp(kind,'arc')&&radius>=25&&any(direction==[-1,1]));
 center=pose(1:2)+direction*radius*[-sin(pose(3));cos(pose(3))];heading1=pose(3)+direction*length/radius;
 finish=center+direction*radius*[sin(heading1);-cos(heading1)];
end
segment=struct('kind',kind,'start',pose(1:2),'finish',finish,'center',center,'radius',radius,...
 'direction',direction,'heading0',pose(3),'heading1',heading1,'length',length,'role',role,'target',target);
end
