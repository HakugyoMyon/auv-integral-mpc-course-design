function spec=sta_scene(kind,seed,plan)
% Environment factory: only sensor and OFFLINE observers read its world.
if nargin<1,kind='single';end;if nargin<2,seed=81001;end
assert(any(strcmp(kind,{'single','vertical','multiple','none','scattered','vertical_wall_wide','vertical_channel'})),'Unknown static scene kind.');
stream=RandStream('mt19937ar','Seed',seed);
n=50+2*(rand(stream)-.5);e=0+1*(rand(stream)-.5);z=14;
objects={struct('kind','sphere','centerNED',[n;e;z],'radius_m',1)};
if strcmp(kind,'vertical')
 n=58;e=0;objects={struct('kind','sphere','centerNED',[n;e;z],'radius_m',.5)};
elseif strcmp(kind,'multiple')
 objects{2}=struct('kind','sphere','centerNED',[92;-3;14],'radius_m',1);
elseif strcmp(kind,'none'),objects={};end
spec=struct('schema','auv.static_experiment.v1','kind',kind,'seed',seed,...
 'world',struct('obstacles',{objects},'terrainFcn',@auv6_terrain,'surfaceDepth_m',0),...
 'launchPrior',struct('operatorVerified',true,'centerNED',[2;0;14],'radius_m',8,'positionErrorBound_m',0),...
 'minimumClearance_m',.5,'computeStopPosix',1794228682,'hardDeadlinePosix',1794229282,...
 'scope','Randomized stationary geometric obstacles initially unknown to map/planner. Explicit operator-verified launch sphere; multislice3D scanning is a new MIL sensing assumption, not a completed mechanical upgrade.');
if any(strcmp(kind,{'scattered','vertical_wall_wide','vertical_channel'}))
 if nargin>=3,[spec.world,spec.fixtureAudit]=sta_validation_world(kind,seed,plan);
 else,[spec.world,spec.fixtureAudit]=sta_validation_world(kind,seed);end
 objects=spec.world.obstacles;
end
% Check declared prior against WORLD only as an offline scenario contract.
for k=1:numel(objects),o=objects{k};assert(norm(o.centerNED-spec.launchPrior.centerNED)>o.radius_m+spec.launchPrior.radius_m);end
assert(auv6_terrain(2,0)-14>spec.launchPrior.radius_m+.07*spec.launchPrior.radius_m);
end

