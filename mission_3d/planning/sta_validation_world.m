function [world,audit]=sta_validation_world(kind,seed,plan)
% OFFLINE fixtures only. Never pass audit/world geometry to navigation/planner.
% The default task is straight outbound [0,0,14] -> [180,0,14], home [0,0,12].
if nargin<1,kind='single';end
if nargin<2,seed=82001;end
assert(any(strcmp(kind,{'single','scattered','vertical_channel','vertical_wall_wide'})));
stream=RandStream('mt19937ar','Seed',seed);
prior=struct('operatorVerified',true,'centerNED',[2;0;14],...
 'radius_m',8,'positionErrorBound_m',0);
goals=[180,0,14;0,0,12];
if nargin>=3
 if isfield(plan,'targets')&&isfield(plan.targets,'positionsNED'),goals=plan.targets.positionsNED;end
 goals=[goals;plan.task.homePose(1:2),plan.task.depth.return_m];
end
protectedRadius=5;objects={};
if strcmp(kind,'single')
 objects={sphere([50+2*(rand(stream)-.5);.4*(rand(stream)-.5);14],.8+.3*rand(stream))};
elseif strcmp(kind,'scattered')
 % First sphere deliberately intersects the nominal outbound centerline.
 objects={sphere([50;0;14],.9)};wanted=6+randi(stream,3)-1;attempts=0;
 while numel(objects)<wanted&&attempts<500
  attempts=attempts+1;
  o=sphere([35+110*rand(stream);-20+40*rand(stream);8+11*rand(stream)],.5+.7*rand(stream));
  if ~protected(o),continue;end
  separated=true;
  for j=1:numel(objects)
   if norm(o.centerNED-objects{j}.centerNED)<o.radius_m+objects{j}.radius_m+6,separated=false;break;end
  end
  if separated,objects{end+1}=o;end
 end
 assert(numel(objects)==wanted,'Could not create separated obstacle fixture.');
elseif strcmp(kind,'vertical_channel')
 % A cross-track low wall is a TEST CANDIDATE, not a qualified vertical case.
 % On chart E in [-20,20], its 7m public chart margin limits horizontal space.
 for e=-12:4:12,objects{end+1}=sphere([58;e;14],.5);end
else
 % Keep the original WIDE chart: a wall blocks horizontal library offsets.
 % Tiny downward offset preserves true centerline intersection (0.45<0.5),
 % yet makes upward avoidance less demanding without changing any budget.
 % Still unqualified until real map and MPC accept an automatically chosen path.
 for e=-16:4:16,objects{end+1}=sphere([58.5;e;14.45],.5);end
end
geometry=zeros(numel(objects),4);
for j=1:numel(objects)
 o=objects{j};assert(protected(o),'Fixture intersects protected launch/home/target volume.');
 geometry(j,:)=[o.centerNED',o.radius_m];
 % Entire obstacle must remain away from physical surface and terrain.
 assert(o.centerNED(3)>o.radius_m+.5);
 assert(auv6_terrain(o.centerNED(1),o.centerNED(2))-o.centerNED(3)>o.radius_m+2);
end
assert(auv6_terrain(2,0)-14>prior.radius_m+.07*prior.radius_m);
if nargin>=3
 nominal=zeros(0,3);
 for k=1:numel(plan.segments)
  seg=plan.segments(k);s=unique([0:.25:seg.length,seg.length]);p=d11_path_eval(seg,s);
  depth=plan.task.depth.transit_m;
  if strcmp(seg.role,'return'),depth=plan.task.depth.return_m;
  elseif isfield(plan.targets,'positionsNED')&&seg.target>=1&&seg.target<=size(plan.targets.positionsNED,1)
   depth=plan.targets.positionsNED(seg.target,3);
  end
  nominal=[nominal;p,repmat(depth,size(p,1),1)];
 end
 nominalBasis='Original segment XY samples with per-role target depths; reference acquisition transients not represented.';
else
 nominal=[(0:.25:180)',zeros(721,1),14*ones(721,1)];
 nominalBasis='Default outbound straight centerline N0..180,E0,D14.';
end
minimum=Inf;conflicts=false(numel(objects),1);centerlineIntersection=false(numel(objects),1);
for j=1:numel(objects)
 distance=min(vecnorm(nominal-objects{j}.centerNED',2,2));
 centerlineIntersection(j)=distance<=objects{j}.radius_m;
 lower=distance-objects{j}.radius_m-sqrt(2)*.205-.125;
 minimum=min(minimum,lower);conflicts(j)=lower<.5;
end
world=struct('obstacles',{objects},'terrainFcn',@auv6_terrain,'surfaceDepth_m',0);
audit=struct('schema','auv.static_offline_fixture.v1','kind',kind,'seed',seed,...
 'obstacleGeometry_N_E_D_radius_m',geometry,'launchPrior',prior,...
 'protectedGoalCentersNED',goals,'protectedGoalRadius_m',protectedRadius,...
 'nominalConflictDetected',any(conflicts),'nominalConflictingObstacleIndices',find(conflicts),...
 'nominalCenterlineIntersectsSphere',any(centerlineIntersection),...
 'centerlineIntersectingObstacleIndices',find(centerlineIntersection),...
 'nominalSampledLowerClearance_m',minimum,'nominalBasis',nominalBasis,...
 'verticalPositiveQualified',false,...
 'scope','Offline scenario design/audit only. Nominal conflict is geometric screening, not executed-body evidence. World goes exclusively to sensor and offline observer; audit must not enter map/planner. Random fixtures are not claimed solvable.');
audit.verticalAdvice='Use auto axis and original uncertainty budgets. Keep the original wide chart: a global E[-20,20] restriction also makes a R25 full return turn infeasible. The vertical_wall_wide fixture adds edge spheres at E +/-16; maximum catalog S offset is 2R(1-cos45)=14.645m. N58.5 and D14.45 preserve actual nominal centerline intersection while favoring upward passage. At .85m/s and .06m/s depth slew, available nominal depth change is only about4.13m before the wall, less capture/trigger latency. This is a feasibility hypothesis, not a vertical positive; require actual free-scan, full uncertainty, MPC and complete-task validation.';
 function okay=protected(o)
  okay=norm(o.centerNED-prior.centerNED)>o.radius_m+prior.radius_m;
  if okay,okay=all(vecnorm(goals-o.centerNED',2,2)>protectedRadius+o.radius_m);end
 end
end
function o=sphere(center,radius)
o=struct('kind','sphere','centerNED',center,'radius_m',radius);
end
