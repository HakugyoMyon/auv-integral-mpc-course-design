function mission=auv_task_normalize(mission)
% Canonical vector/table shapes survive JSON save/load and manual column input.
t=mission.task;t.startPose=reshape(t.startPose,1,3);t.homePose=reshape(t.homePose,1,3);
t.chart.operatingBounds=reshape(t.chart.operatingBounds,1,4);
if isempty(t.chart.knownCircles),t.chart.knownCircles=zeros(0,3);end
if strcmp(t.type,'survey'),t.survey.centerNE=reshape(t.survey.centerNE,1,2);end
if strcmp(t.type,'inspection')&&isvector(t.inspection.polylineNE),t.inspection.polylineNE=reshape(t.inspection.polylineNE,[],2);end
if strcmp(t.type,'transit'),t.transit.destinationNED=reshape(t.transit.destinationNED,1,3);end
if strcmp(t.type,'visit')
 if isvector(t.visit.pointsNED),t.visit.pointsNED=reshape(t.visit.pointsNED,1,3);end
 t.visit.priority=t.visit.priority(:);
end
mission.task=t;
end
