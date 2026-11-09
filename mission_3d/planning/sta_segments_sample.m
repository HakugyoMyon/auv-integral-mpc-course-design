function sample=sta_segments_sample(segments,depthSchedule,spacing)
% Uniform horizontal arc-length sampling, exact XY line/arc geometry.
lengths=[segments.length];edges=[0,cumsum(lengths)];total=edges(end);
s=unique([0:spacing:total,total,edges]);points=zeros(numel(s),3);yaw=zeros(numel(s),1);segment=zeros(numel(s),1);
for k=1:numel(s)
 index=find(s(k)<=edges(2:end)+1e-9,1,'first');if isempty(index),index=numel(segments);end
 part=segments(index);distance=min(part.length,max(0,s(k)-edges(index)));
 if strcmp(part.kind,'line')
  p=part.start+distance*[cos(part.heading0);sin(part.heading0)];heading=part.heading0;
 else
  heading=part.heading0+part.direction*distance/part.radius;
  p=part.center+part.direction*part.radius*[sin(heading);-cos(heading)];
 end
 points(k,1:2)=p';yaw(k)=heading;segment(k)=index;
end
assert(size(depthSchedule,2)==2&&all(diff(depthSchedule(:,1))>0));
points(:,3)=interp1(depthSchedule(:,1),depthSchedule(:,2),s,'linear','extrap');
depthSlope=zeros(size(s));
for k=1:numel(s)
 index=find(s(k)<depthSchedule(2:end,1)-1e-9,1,'first');if isempty(index),index=size(depthSchedule,1)-1;end
 depthSlope(k)=diff(depthSchedule(index:index+1,2))/diff(depthSchedule(index:index+1,1));
end
sample=struct('s_m',s(:),'pointsNED',points,'yaw_rad',yaw,'depthSlope',depthSlope(:),'segment',segment);
end
