function result=sta_gui_contract(out)
% Boundary/persistence checks, not a replacement for GUI full-mission tests.
root=fileparts(mfilename('fullpath'));if ~isfolder(out),mkdir(out);end
names={'random82001','random82002','vertical_wall','inspection_transit'};checks=0;
for k=1:numel(names)
 [c,s]=sta_case_config(names{k},root,'whole_ball_free_fusion');
 [normalized,p,actual,driver]=sta_gui_prepare(c,'static_unknown',s.kind,s.seed);
 assert(isequaln(actual.world,s.world)&&strcmp(normalized.execution.environment,'static_unknown')&&normalized.execution.avoidanceEnabled);checks=checks+1;
 assert(p.verticalScreen.referenceSlewScreenComplete&&actual.computeStopPosix>posixtime(datetime('now','TimeZone','UTC')));
 if strcmp(c.task.type,'inspection'),assert(strcmp(driver,'sta_run_terrain'));else,assert(strcmp(driver,'sta_run'));end;checks=checks+1;
 d=sta_gui_definition(c,'static_unknown',s.kind,s.seed);file=fullfile(out,[names{k},'.json']);sta_gui_save(d,file);loaded=sta_gui_load(file);
 assert(isequaln(d,loaded)&&~isfield(loaded,'world')&&~isfield(loaded,'obstacles'));checks=checks+1;
 [base,~,spec,baseDriver]=sta_gui_prepare(c,'obstacle_free',s.kind,s.seed);
 assert(strcmp(base.execution.environment,'obstacle_free')&&~base.execution.avoidanceEnabled&&isempty(spec)&&strcmp(baseDriver,'auv_task_run'));checks=checks+1;
end
c=auv_task_load(fullfile(root,'baseline_v4','missions','visit.json'));
f=fullfile(out,'legacy.json');fid=fopen(f,'w');fprintf(fid,'%s',jsonencode(c));fclose(fid);d=sta_gui_load(f);assert(strcmp(d.mode,'obstacle_free'));checks=checks+1;
bad=struct('artifactRole','OFFLINE_SIMULATOR_GROUND_TRUTH_ONLY_NOT_A_NAVIGATION_MAP','world',struct());f=fullfile(out,'not_a_task.json');fid=fopen(f,'w');fprintf(fid,'%s',jsonencode(bad));fclose(fid);
rejected=false;try,sta_gui_load(f);catch,rejected=true;end;assert(rejected);checks=checks+1;
rejected=false;try,sta_gui_prepare(c,'static_unknown','scattered',1.5);catch,rejected=true;end;assert(rejected);checks=checks+1;
c.task.depth.transit_m=13;rejected=false;try,sta_gui_prepare(c,'static_unknown','scattered',82002);catch,rejected=true;end;assert(rejected);checks=checks+1;
publicMap=struct('occupiedCentersNED',[(0:9999)'/100,zeros(10000,2)],'acceptedScans',3);x=zeros(18,1);x(4)=1;randomBefore=rng;
s=sta_live_status(1,x,0,1,false,'none',true,publicMap);assert(size(s.observedMapNED,1)<=1500&&max(s.observedMapNED(:,1))<=60&&isequal(randomBefore,rng));checks=checks+1;
s=sta_live_status(1.1,x,0,1,false,'none',true,publicMap);assert(~isfield(s,'observedMapNED')&&~isfield(s,'truth')&&~isfield(s,'world'));checks=checks+1;
result=struct('passed',true,'checks',checks,'scope','Four preset world/driver/flags/deadline contracts, static definition JSON roundtrips, legacy task import, truth-map rejection, invalid seed and launch-depth rejection; bounded, deterministic observed-only telemetry. Does not establish full missions, GUI widgets or arbitrary-case success.');
fid=fopen(fullfile(out,'GUI_CONTRACT.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
