function result=sta_fusion_world_contract(outdir)
% Experimental OFFLINE property checks against independent exact ball geometry.
% World enters map ONLY through sta_sensor. Truth is used after query for audit.
s=RandStream('mt19937ar','Seed',953117);c=sta_sensor_config();c.dropoutProbability=0;
sceneCount=10;freeCount=0;falseFree=0;overrides=0;queries=0;minimumSlack=Inf;
details=zeros(sceneCount,5);
for scene=1:sceneCount
 x=zeros(18,1);x(3)=14;x(4)=1;objects=cell(1,4);
 for k=1:3
  objects{k}=struct('kind','sphere','centerNED',[22+22*rand(s);-15+30*rand(s);6+18*rand(s)],'radius_m',.3+2*rand(s));
 end
 centerDepth=10+10*rand(s);halfHeight=1+3*rand(s);
 objects{4}=struct('kind','cylinder','centerNE',[25+18*rand(s);-13+26*rand(s)],'radius_m',.5+2*rand(s),...
  'depthLimits_m',[centerDepth-halfHeight,centerDepth+halfHeight]);
 world=struct('obstacles',{objects},'bottomDepth_m',30,'surfaceDepth_m',0);
 packet=sta_sensor(world,x,scene,c,s);
 direction=randn(s,3,1);direction=direction/norm(direction);error=(.1+.7*rand(s))*direction;
 axis=randn(s,3,1);axis=axis/norm(axis);angle=.02*rand(s);
 nav=struct('positionNED',x(1:3)+error,'quaternion',[cos(angle/2);axis*sin(angle/2)],...
  'positionErrorBound_m',.8,'attitudeErrorBound_rad',.02,'valid',true,'time',scene);
 assert(norm(error)<=nav.positionErrorBound_m+1e-12&&angle<=nav.attitudeErrorBound_rad+1e-12);
 map=sta_map_init([],c);[map,event]=sta_map_update(map,packet,nav);assert(event.accepted);
 points=[60*rand(s,350,1),-35+70*rand(s,350,1),2+26*rand(s,350,1)];
 % Additional near-front probes are generated from NAV only, never world.
 R=auv6_rotation(nav.quaternion);front=(nav.positionNED+R*[linspace(8,18,30);zeros(2,30)])';
 points=[points;front];margin=[.5+2*rand(s,350,1);.5*ones(30,1)];
 initial=sta_map_query_fusion(map,points,margin);
 candidates=find(initial.free,5,'first');
 % Artificially enlarge possible old endpoint uncertainty. This changes only
 % the experimental map outer bound, never the world or free certificates.
 if ~isempty(candidates)
  map.occupiedCentersNED=[map.occupiedCentersNED;points(candidates,:)];
  map.occupiedError_m=[map.occupiedError_m;5*ones(numel(candidates),1)];
 end
 q=sta_map_query_fusion(map,points,margin);legacy=sta_map_query(map,points,margin);
 sceneOverrides=sum(q.free&~legacy.free);overrides=overrides+sceneOverrides;
 count=sum(q.free);freeCount=freeCount+count;queries=queries+size(points,1);bad=0;
 for k=find(q.free)'
  slack=exactBallSlack(points(k,:)',margin(k),world);minimumSlack=min(minimumSlack,slack);
  if slack< -1e-7,bad=bad+1;end
 end
 falseFree=falseFree+bad;
 behind=(nav.positionNED+R*[-10;0;0])';qBehind=sta_map_query_fusion(map,behind,.5);assert(qBehind.unknown,'Outside scan FOV must stay unknown.');
 blank=packet;blank.rayState(:)=0;blank.time=scene+.1;nav.time=blank.time;
 emptyMap=sta_map_init([],c);emptyMap=sta_map_update(emptyMap,blank,nav);
 unknown=sta_map_query_fusion(emptyMap,front,.5);assert(all(unknown.unknown),'Invalid rays must not produce free space.');
 details(scene,:)=[scene,size(points,1),count,sceneOverrides,bad];
end
prior=struct('operatorVerified',true,'centerNED',[0;0;14],'radius_m',6,'positionErrorBound_m',0);
map=sta_map_init(prior,c);map.occupiedCentersNED=[0,0,14];map.occupiedError_m=1;
q=sta_map_query_fusion(map,[0,0,14],.5);assert(q.occupied,'Prior alone must not override occupied guard.');
assert(freeCount>0&&overrides>0,'Property test did not exercise observed-free overrides.');
assert(falseFree==0,'Fusion created %d false-free balls.',falseFree);
result=struct('passed',true,'seed',953117,'scenes',sceneCount,'queries',queries,'certified_free_balls',freeCount,...
 'possible_guard_overrides',overrides,'false_free_balls',falseFree,'minimum_exact_free_ball_slack_m',minimumSlack,...
 'capture_position_bound_m',.8,'capture_attitude_bound_rad',.02,'query_radius_range_m',[.5,2.5],...
 'scene_details',details,'scope','Experimental finite randomized offline property check, not a proof or approval to deploy. Static sphere/finite vertical cylinder, surface and flat bottom; bounded sensor errors and finite angular cells. Exact ball/world validation uses truth only after map query. No prior in main randomized scenes. Statistical joint validity, real acoustic false negatives and dynamic worlds are outside scope.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'fusion_world_contract.json'),'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));
end
disp(result);
end

function slack=exactBallSlack(p,r,world)
slack=min(p(3)-world.surfaceDepth_m,world.bottomDepth_m-p(3))-r;
for k=1:numel(world.obstacles)
 o=world.obstacles{k};
 if strcmp(o.kind,'sphere'),d=norm(p-o.centerNED)-o.radius_m;
 else
  radial=max(0,norm(p(1:2)-o.centerNE)-o.radius_m);
  vertical=max([o.depthLimits_m(1)-p(3),p(3)-o.depthLimits_m(2),0]);
  d=hypot(radial,vertical);
  % For centers inside the finite cylinder d=0 and radius>0, correctly bad.
 end
 slack=min(slack,d-r);
end
end
