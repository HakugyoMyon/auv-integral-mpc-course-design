function result=sta_downlooking_contract()
% Synthetic contracts only; not a complete survey/inspection mission test.
x=zeros(18,1);x(3)=10;x(4)=1;terrain=@(n,e)20+zeros(size(n+e));
empty=struct('obstacles',struct([]));alt=auv6_altimeter_config();alt.dropoutProbability=0;
spec=struct('alongHalfAngle_rad',deg2rad(25),'crossHalfAngle_rad',deg2rad(25),'maximumRange_m',30);
pay=d11_payload_config(spec);pay.dropoutProbability=0;pay.mount=zeros(3,1);
s1=RandStream('mt19937ar','Seed',781);s2=RandStream('mt19937ar','Seed',781);
a=auv6_altimeter(x,0,alt,terrain,s1,false);a=rmfield(a,{'attitudeAtSample','depthAtSample'});
b=sta_altimeter(x,0,alt,terrain,s2,false,empty);
assert(isequaln(a,rmfield(b,{'occlusionChecked','occluded','occlusionScope'}))&&isequal(s1.State,s2.State));
s1=RandStream('mt19937ar','Seed',782);s2=RandStream('mt19937ar','Seed',782);
a=d11_payload_sensor(x,0,pay,terrain,s1,false);b=sta_payload_sensor(x,0,pay,terrain,s2,false,empty);
assert(isequaln(a,rmfield(b,{'occlusionChecked','occluded','occlusionScope'}))&&isequal(s1.State,s2.State));
R=auv6_rotation(x(4:7));point=x(1:3)+R*alt.mount+3*R*alt.beams(:,1);
world=struct('obstacles',struct('kind','sphere','centerNED',point,'radius_m',.25));
a=sta_altimeter(x,0,alt,terrain,RandStream('mt19937ar','Seed',1),false,world);
assert(a.occluded&&~a.valid&&~a.beamValid(1)&&isnan(a.ranges(1)));
world.obstacles=struct('kind','sphere','centerNED',[2;1;17],'radius_m',.2);
a=sta_payload_sensor(x,0,pay,terrain,RandStream('mt19937ar','Seed',2),false,world);
assert(a.occluded&&~a.valid,'Interior obstacle between the five rays must invalidate footprint.');
world.obstacles=struct('kind','sphere','centerNED',[100;100;15],'radius_m',1);
a=sta_payload_sensor(x,0,pay,terrain,RandStream('mt19937ar','Seed',3),false,world);assert(a.valid&&~a.occluded);
world.obstacles=struct('kind','cylinder','centerNE',[2;1],'radius_m',.25,'depthLimits_m',[16,18]);
a=sta_payload_sensor(x,0,pay,terrain,RandStream('mt19937ar','Seed',4),false,world);assert(a.occluded&&~a.valid);
world.obstacles=struct('kind','sphere','centerNED',[0;0;25],'radius_m',.5);
a=sta_payload_sensor(x,0,pay,terrain,RandStream('mt19937ar','Seed',5),false,world);assert(a.valid&&~a.occluded,'Obstacle beyond seabed must not occlude.');
assert(~isfield(a,'world')&&~isfield(a,'obstacles')&&~isfield(a,'truth'));
result=struct('passed',true,'checks',8,'scope','Synthetic empty-world/RNG equivalence, beam interception, full-footprint interior obstruction, distant and behind-bottom rejection, finite cylinder and packet schema. Full MIL qualification remains separate.');
end
