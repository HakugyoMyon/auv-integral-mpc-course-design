function [progress,error,point,heading,raw]=d11_path_project(position,segment,previousProgress)
if strcmp(segment.kind,'line')
 tangent=[cos(segment.heading0);sin(segment.heading0)];raw=dot(position-segment.start,tangent);
else
 d=segment.direction;angle=atan2(d*(position(1)-segment.center(1)),d*(segment.center(2)-position(2)));
 old=segment.heading0+d*previousProgress/segment.radius;
 angle=old+atan2(sin(angle-old),cos(angle-old));raw=d*segment.radius*(angle-segment.heading0);
end
progress=min(max(raw,0),segment.length);[p,heading]=d11_path_eval(segment,progress);point=p';
error=dot(position-point,[-sin(heading);cos(heading)]);
end
