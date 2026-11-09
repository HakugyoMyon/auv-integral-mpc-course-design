function auv_task_check_capability(task)
% Resource and numerical checks BEFORE any route-geometry allocation.
assert(all(isfinite(task.homePose)),'Home pose must be finite.');
assert(isfinite(task.chart.clearance_m)&&task.chart.clearance_m>=7,'Chart clearance must be finite and at least7m.');
assert(isfinite(task.minimumTurnRadius_m)&&task.minimumTurnRadius_m>=25&&task.minimumTurnRadius_m<=200,'Supported nominal turn radius is25..200m.');
d=task.depth;assert(isscalar(d.maximumRate_m_s)&&d.maximumRate_m_s>0&&d.maximumRate_m_s<=.06,'Invalid depth-reference slew limit.');
if isfield(d,'workDepthBounds_m')
 e=d.workDepthBounds_m;assert(isnumeric(e)&&numel(e)==2&&all(isfinite(e))&&e(1)>=4&&e(2)>=e(1),'Invalid declared work-depth interval.');
end
p=task.payload;
assert(all(isfinite([p.crossHalfAngle_rad,p.alongHalfAngle_rad,p.maximumRange_m,p.nominalNavigationMargin_m,p.gridResolution_m,p.minimumObservations])),'Nonfinite payload settings.');
assert(p.crossHalfAngle_rad>0&&p.crossHalfAngle_rad<pi/2&&p.alongHalfAngle_rad>0&&p.alongHalfAngle_rad<pi/2&&p.maximumRange_m>0&&p.nominalNavigationMargin_m>=0&&p.gridResolution_m>0,'Invalid observation geometry.');
assert(p.minimumObservations>=3&&p.minimumObservations<=50&&p.minimumObservations==floor(p.minimumObservations),'Observation support count must be integer3..50.');
if strcmp(task.type,'survey')
 s=task.survey;assert(all(isfinite([s.centerNE,s.length_m,s.width_m,s.bearing_rad,s.maximumLineSpacing_m,s.requiredCoverage])),'Nonfinite survey settings.');
 margin=p.nominalNavigationMargin_m+sqrt(2)*p.gridResolution_m/2+.3;
 spacing=min(s.maximumLineSpacing_m,2*(d.workAltitude_m*tan(p.crossHalfAngle_rad)-margin));
 assert(spacing>0&&ceil(s.width_m/spacing)<=100,'Survey needs too many lanes or has no positive observation width (limit100 lanes).');
elseif strcmp(task.type,'inspection')
 n=task.inspection.minimumObservations;assert(isfinite(n)&&n>=3&&n<=50&&n==floor(n),'Inspection support count must be integer3..50.');
elseif strcmp(task.type,'visit')
 assert(~isempty(task.visit.pointsNED)&&all(isfinite(task.visit.pointsNED),'all')&&all(isfinite(task.visit.priority)),'Visit targets and priorities must be nonempty and finite.');
end
if any(strcmp(task.type,{'visit','transit'}))
 if strcmp(task.type,'visit'),s=task.visit;else,s=task.transit;end
 assert(all(isfinite([s.captureRadius_m,s.maximumBoundDistance_m,s.depthTolerance_m]))&&s.captureRadius_m>0&&s.captureRadius_m<=3&&s.maximumBoundDistance_m>=s.captureRadius_m&&s.maximumBoundDistance_m<=5&&s.depthTolerance_m>0&&s.depthTolerance_m<=.5,'Supported fly-through tolerance is horizontal<=3m, confidence bound<=5m and depth<=0.5m.');
end
% A necessary vertical-time lower bound prevents the legacy connector from
% allocating arbitrarily many preparation circles for impossible requests.
depths=d.return_m;
if strcmp(task.type,'visit'),depths=[depths;task.visit.pointsNED(:,3)];
elseif strcmp(task.type,'transit'),depths=[depths;task.transit.destinationNED(3)];
elseif isfield(d,'workDepthBounds_m'),depths=[depths;d.workDepthBounds_m(:)];end
delta=max(abs(depths-d.transit_m));
assert(delta==0||delta/d.maximumRate_m_s+5<=task.maximumTime_s,...
 'Declared depths require more reference-slew time than the whole mission budget; no route geometry will be allocated.');
end
