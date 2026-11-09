function result=sta_fusion_contract()
% OFFLINE experimental whole-volume free/possible-occupancy set fusion.
c=sta_sensor_config();c.dropoutProbability=0;x=zeros(18,1);x(3)=14;x(4)=1;
world=struct('obstacles',struct([]),'bottomDepth_m',30,'surfaceDepth_m',0);
nav=struct('positionNED',[0;0;14],'quaternion',[1;0;0;0],'positionErrorBound_m',.05,'attitudeErrorBound_rad',0,'valid',true,'time',1);
p=sta_sensor(world,x,1,c,RandStream('mt19937ar','Seed',883));map=sta_map_init([],c);map=sta_map_update(map,p,nav);
% A possible old endpoint location overlaps genuinely observed free volume.
map.occupiedCentersNED=[12,0,14];map.occupiedError_m=2;
old=sta_map_query(map,[12,0,14],.5);new=sta_map_query_fusion(map,[12,0,14],.5);
assert(old.occupied&&new.free,'Full-ball observed-free must reject possible guard overlap.');
unknown=sta_map_query_fusion(map,[0,20,14],.5);assert(unknown.unknown);
% A prior alone is not a sensor-certified override of a guard conflict.
prior=struct('operatorVerified',true,'centerNED',[0;0;14],'radius_m',6,'positionErrorBound_m',0);
onlyPrior=sta_map_init(prior,c);onlyPrior.occupiedCentersNED=[0,0,14];onlyPrior.occupiedError_m=1;
a=sta_map_query_fusion(onlyPrior,[0,0,14],.2);assert(a.occupied);
% Center-ray clearance must not substitute for whole-body containment.
a=sta_map_query_fusion(map,[12,20,14],3);assert(~a.free);
% Real blocking geometry remains outside every correctly formed free prefix.
world.obstacles=struct('kind','sphere','centerNED',[25;0;14],'radius_m',2);
p=sta_sensor(world,x,2,c,RandStream('mt19937ar','Seed',884));nav.time=2;
blocked=sta_map_init([],c);blocked=sta_map_update(blocked,p,nav);
a=sta_map_query_fusion(blocked,[25,0,14;35,0,14],.5);assert(~any(a.free));
% Missing rays do not generate certificates, even at a conflicting guard.
invalid=p;invalid.time=3;invalid.rayState(:)=0;nav.time=3;
blank=sta_map_init([],c);blank=sta_map_update(blank,invalid,nav);
blank.occupiedCentersNED=[12,0,14];blank.occupiedError_m=2;
a=sta_map_query_fusion(blank,[12,0,14],.5);assert(a.occupied);
map.capacityExceeded=true;a=sta_map_query_fusion(map,[12,0,14],.5);assert(a.unknown);
result=struct('passed',true,'checks',7,'scope','EXPERIMENT ONLY. Whole-ball observed-free versus possible guard exclusion, unknown/prior/nonzero-body/actual sphere occlusion/invalid ray/capacity adversaries. Not approval to deploy or claim real sensor certainty.');
end
