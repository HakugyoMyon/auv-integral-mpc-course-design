function result=sta_projection_contract(outdir)
% Synthetic local route/pointer cases; not a navigation or mission claim.
line=d11_segment([0;0;0],'line',100,Inf,0,'transit',0);route=struct('segments',line);
pointer=struct('segment',1,'progress',10,'filteredSideslip',.073,'bottomDepth',24,'lastBottomTime',2);
[p,a]=sta_project_master(route,pose([35;2;14],0),pointer);
assert(a.accepted&&p.segment==1&&abs(p.progress-35)<1e-9&&abs(a.distance_m-2)<1e-9);
assert(p.filteredSideslip==pointer.filteredSideslip&&p.bottomDepth==pointer.bottomDepth&&p.lastBottomTime==pointer.lastBottomTime);
pointer.progress=40;[p,a]=sta_project_master(route,pose([35;0;14],0),pointer);assert(a.accepted&&p.progress>=40);
[p,a]=sta_project_master(route,pose([20;0;14],0),pointer);assert(~a.accepted&&isequal(p,pointer));
pointer.progress=10;pointer.projectionWindow_m=30;[p,a]=sta_project_master(route,pose([65;0;14],0),pointer);assert(~a.accepted&&isequal(p,pointer));
pointer=rmfield(pointer,'projectionWindow_m');[p,a]=sta_project_master(route,pose([35;0;14],pi),pointer);assert(~a.accepted&&isequal(p,pointer));
back=d11_segment([100;0;pi],'line',100,Inf,0,'transit',0);again=d11_segment([0;0;0],'line',100,Inf,0,'transit',0);cross=struct('segments',[line,back,again]);
[p,a]=sta_project_master(cross,pose([50;0;14],0),pointer);assert(a.accepted&&p.segment==1&&abs(p.progress-50)<1e-9);
circle=d11_segment([0;0;0],'arc',2*pi*25,25,1,'transit',0);[point,heading]=d11_path_eval(circle,120);circleRoute=struct('segments',circle);pointer.segment=1;pointer.progress=30;
[p,a]=sta_project_master(circleRoute,pose([point(:);14],heading),pointer);assert(a.accepted&&abs(p.progress-120)<1e-7&&a.advance_m<=100);
result=struct('passed',true,'cases',7,'scope','Synthetic nearest/monotone/window/heading/crossing/lap contracts. No truth access, pose mutation, ledger reset, connection certification or whole-task qualification.');
if nargin>0
 if ~isfolder(outdir),mkdir(outdir);end
 fid=fopen(fullfile(outdir,'projection_contract.json'),'w');fprintf(fid,'%s',jsonencode(result,PrettyPrint=true));fclose(fid);
end
end
function x=pose(position,yaw)
x=zeros(18,1);x(1:3)=position;x(4:7)=[cos(yaw/2);0;0;sin(yaw/2)];x(8)=.8;
end
