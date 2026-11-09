function result=sta_perception_contract(outdir)
% Synthetic geometry/error-contract checks, not full controlled navigation.
c=sta_sonar_config();c.dropoutProbability=0;s=RandStream('mt19937ar','Seed',82001);
x=zeros(18,1);x(4)=1;x(3)=14;world=struct('obstacles',struct([]),'bottomDepth_m',26,'surfaceDepth_m',0);
nav=struct('positionNED',[0;0;14],'quaternion',[1;0;0;0],'positionErrorBound_m',.05,...
 'attitudeErrorBound_rad',deg2rad(.1),'valid',true,'time',1);
packet=sta_sonar(world,x,1,c,s);assert(~any(isfield(packet,{'world','obstacles','truth','centerNED','radius_m'})));
map=sta_map_init([],c);[map,event]=sta_map_update(map,packet,nav);assert(event.accepted);
q=sta_map_query(map,[12,0,14;12,40,14;0,0,14],[.5;.1;.1]);assert(isequal(q.state,int8([0;-1;-1])));
qsmall=sta_map_query(map,[12,20,14],.1);qbody=sta_map_query(map,[12,20,14],3);assert(qsmall.free&&~qbody.free);
q=sta_map_query(map,[20,0,1;20,0,26],.2);assert(~any(q.free));
old=map;bad=nav;bad.valid=false;[map,event]=sta_map_update(map,packet,bad);assert(~event.accepted&&isequaln(map,old));
late=nav;late.time=1.3;latePacket=packet;latePacket.time=1.2;[map,event]=sta_map_update(map,latePacket,late);assert(~event.accepted&&strcmp(event.reason,'pose_not_at_capture_time'));
[map,event]=sta_map_update(map,packet,nav);assert(~event.accepted&&strcmp(event.reason,'duplicate_or_out_of_order'));
expectedOriginError=nav.positionErrorBound_m+2*norm(c.mountFRD)*sin(nav.attitudeErrorBound_rad/2);
assert(abs(map.frames{1}.positionErrorBound_m-expectedOriginError)<1e-12);
world.obstacles=struct('kind','sphere','centerNED',[25;0;14],'radius_m',2);
packet=sta_sonar(world,x,2,c,s);nav.time=2;blocked=sta_map_init([],c);[blocked,event]=sta_map_update(blocked,packet,nav);assert(event.accepted);
q=sta_map_query(blocked,[25,0,14;35,0,14;25,5,14],.2);assert(~q.free(1)&&~q.free(2)&&q.free(3));
first=sta_map_query(blocked,[22,0,14],.2);assert(first.occupied);
% Fixed static memory has no target-speed/time-growth field.
assert(~isfield(blocked,'maxTargetSpeed')&&~isfield(blocked,'track'));
world.obstacles=struct('kind','cylinder','centerNE',[25;0],'radius_m',2,'depthLimits_m',[12,16]);
packet=sta_sonar(world,x,3,c,s);nav.time=3;cylinder=sta_map_init([],c);[cylinder,event]=sta_map_update(cylinder,packet,nav);assert(event.accepted);
q=sta_map_query(cylinder,[25,0,14;25,0,8],.2);assert(~q.free(1)&&q.free(2));
prior=struct('operatorVerified',true,'centerNED',[0;0;14],'radius_m',6,'positionErrorBound_m',0);
launch=sta_map_init(prior,c);q=sta_map_query(launch,[0,0,14;5.8,0,14;8,0,14],.4);assert(isequal(q.state,int8([0;-1;-1])));
% Deliberately off-center tiny sphere still affects a whole angular cell.
world.obstacles=struct('kind','sphere','centerNED',[25;.2;14.2],'radius_m',.1);
packet=sta_sonar(world,x,4,c,s);[~,a]=min(abs(c.azimuth_rad));[~,e]=min(abs(c.elevation_rad));
assert(packet.rayState(e,a)==1&&packet.range_m(e,a)<26&&packet.angularCellGuard_m>0);
result=struct('passed',true,'checks',15,'beamCount',numel(packet.rayState),'angularGuard_m',packet.angularCellGuard_m,...
 'scope','Synthetic finite-angular-cell conservative geometry, whole-ball free certificates, occlusion, water boundaries, launch prior, capture timing and mount/pose error handling. No acoustic hardware, global safe navigation, control or statistical confidence guarantee.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'perception_contract.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
end
