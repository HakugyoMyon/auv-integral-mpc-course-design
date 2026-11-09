function map=sta_map_init(prior,c)
% Only an explicit operator-verified launch volume, never world geometry.
if nargin<1||isempty(prior),prior=struct([]);end
if nargin<2,c=sta_sonar_config();end
if ~isempty(prior)
 assert(isfield(prior,'operatorVerified')&&isequal(prior.operatorVerified,true));
 assert(numel(prior.centerNED)==3&&all(isfinite(prior.centerNED))&&isscalar(prior.radius_m)&&prior.radius_m>0);
 if ~isfield(prior,'positionErrorBound_m'),prior.positionErrorBound_m=0;end
 assert(prior.positionErrorBound_m>=0&&prior.positionErrorBound_m<prior.radius_m);
end
map=struct('schema','auv.static_map.v1','configuration',c,'prior',prior,'frames',{{}},...
 'occupiedCentersNED',zeros(0,3),'occupiedError_m',zeros(0,1),'lastPacketTime',-Inf,...
 'acceptedScans',0,'capacityExceeded',false,...
 'scope','Static finite-cell guard exclusions plus observed free-prefix certificates. Unknown remains unknown; no time-growing motion envelope or truth input. Safety is conditional on declared sensing and navigation error bounds.');
end
