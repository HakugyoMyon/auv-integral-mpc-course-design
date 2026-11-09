function [position,heading]=d11_path_eval(segment,distance)
distance=min(max(distance,0),segment.length);
if strcmp(segment.kind,'line')
 heading=segment.heading0+zeros(size(distance));position=segment.start'+distance(:)*[cos(heading(1)),sin(heading(1))];
else
 heading=segment.heading0+segment.direction*distance/segment.radius;
 position=segment.center'+segment.direction*segment.radius*[sin(heading(:)),-cos(heading(:))];
end
end
