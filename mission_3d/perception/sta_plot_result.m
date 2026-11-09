function files=sta_plot_result(file,outdir)
% OFFLINE illustration only: archived truth never feeds navigation/planning.
% Clearance curves are sampled archived states, not continuous-time proofs.
if nargin<2,outdir=fullfile(fileparts(file),'figures');end
if ~isfolder(outdir),mkdir(outdir);end
z=load(file,'h','plan','staticSpec','report');h=z.h;world=z.staticSpec.world;
blue=[.08,.35,.64];orange=[.88,.36,.08];teal=[0,.53,.49];gray=[.45,.49,.54];
fig=figure('Visible','off','Color','w','Position',[50,50,1400,900]);closer=onCleanup(@()close(fig));
ax=axes(fig);hold(ax,'on');grid(ax,'on');box(ax,'on');
actual=plot3(ax,h.truth(:,1),h.truth(:,2),h.truth(:,3),'-','Color',blue,'LineWidth',2,'DisplayName','Actual / 真实轨迹');
estimated=plot3(ax,h.estimate(:,1),h.estimate(:,2),h.estimate(:,3),'--','Color',teal,'LineWidth',1,'DisplayName','Estimated / 导航估计');
detour=h.truth(:,1:3);detour(~h.staticActive,:)=NaN;
local=plot3(ax,detour(:,1),detour(:,2),detour(:,3),'-','Color',orange,'LineWidth',3,'DisplayName','Avoidance active / 绕行');
[u,v,w]=sphere(28);obstacleHandle=gobjects(0);
for k=1:numel(world.obstacles)
 if iscell(world.obstacles),o=world.obstacles{k};else,o=world.obstacles(k);end
 if strcmp(o.kind,'sphere')
  s=surf(ax,o.centerNED(1)+o.radius_m*u,o.centerNED(2)+o.radius_m*v,o.centerNED(3)+o.radius_m*w,...
   'FaceColor',[.72,.20,.19],'FaceAlpha',.42,'EdgeColor','none');
 elseif strcmp(o.kind,'cylinder')
  [a,b,c]=cylinder(o.radius_m,40);c=o.depthLimits_m(1)+diff(o.depthLimits_m)*c;
  s=surf(ax,a+o.centerNE(1),b+o.centerNE(2),c,'FaceColor',[.72,.20,.19],'FaceAlpha',.42,'EdgeColor','none');
  fill3(ax,a(1,:)+o.centerNE(1),b(1,:)+o.centerNE(2),c(1,:),[.72,.20,.19],'FaceAlpha',.42,'EdgeColor','none','HandleVisibility','off');
  fill3(ax,a(2,:)+o.centerNE(1),b(2,:)+o.centerNE(2),c(2,:),[.72,.20,.19],'FaceAlpha',.42,'EdgeColor','none','HandleVisibility','off');
 else,error('Unsupported archived obstacle geometry.');end
 if isempty(obstacleHandle),s.DisplayName='Static obstacle / 障碍真值';obstacleHandle=s;else,s.HandleVisibility='off';end
end
targets=z.plan.targets;targetHandle=gobjects(0);targetLabel='Task points / 任务点';
if isfield(targets,'positionsNED'),p=targets.positionsNED;
elseif isfield(targets,'positionsNE'),p=[targets.positionsNE,repmat(z.plan.task.depth.transit_m,size(targets.positionsNE,1),1)];
elseif isfield(targets,'centersNE'),p=[targets.centersNE,repmat(z.plan.task.depth.transit_m,size(targets.centersNE,1),1)];
else,p=zeros(0,3);end
if ~isfield(targets,'positionsNED'),targetLabel='Task NE (depth schematic) / 点位NE（示意深度）';end
if ~isempty(p)
 targetHandle=scatter3(ax,p(:,1),p(:,2),p(:,3),45,[.53,.29,.68],'d','filled','DisplayName',targetLabel);
end
home=z.plan.task.homePose;homeHandle=scatter3(ax,home(1),home(2),z.plan.task.depth.return_m,130,[.12,.12,.15],'p','filled','DisplayName','Home / 返航点');
scatter3(ax,h.truth(1,1),h.truth(1,2),h.truth(1,3),70,blue,'o','filled','HandleVisibility','off');
scatter3(ax,h.truth(end,1),h.truth(end,2),h.truth(end,3),70,blue,'s','filled','HandleVisibility','off');
set(ax,'ZDir','reverse','FontSize',11);xlabel(ax,'North N / m');ylabel(ax,'East E / m');zlabel(ax,'Depth D / m (down positive)');
axis(ax,'equal');view(ax,40,24);
legend(ax,[actual,estimated,local,obstacleHandle,targetHandle,homeHandle],'Location','eastoutside','Interpreter','none');
title(ax,{'Unknown static obstacles: archived 3D execution / 未知静态障碍三维执行',...
 sprintf('Task %s | abort=%s | %.1f s | switches %d',z.report.task,z.report.abort,z.report.duration_s,z.report.static_avoidance_switches)},'Interpreter','none');
subtitle(ax,'Obstacle truth is shown only for offline interpretation; it was not provided to the planner.','FontSize',10,'Color',gray);
set(findall(fig,'-property','FontName'),'FontName','Microsoft YaHei');
files.trajectory=fullfile(outdir,'static_trajectory_3d.png');exportgraphics(fig,files.trajectory,'Resolution',180);
clear closer;
% Independent sampled truth clearances. Include positive infinity as NaN for
% an empty obstacle world, rather than drawing an arbitrary large clearance.
obstacle=nan(size(h.t));terrain=nan(size(h.t));positionError=vecnorm(h.truth(:,1:3)-h.estimate(:,1:3),2,2);
if isfield(world,'terrainFcn'),terrainFcn=world.terrainFcn;
elseif isfield(world,'bottomDepth_m'),bottom=world.bottomDepth_m;terrainFcn=@(n,e)bottom+zeros(size(n+e));
else,error('Archived world has no declared bottom model.');end
for k=1:numel(h.t)
 d=sta_world_clearance(h.truth(k,:)',world);if isfinite(d),obstacle(k)=d;end
 terrain(k)=auv6_clearance_bound(h.truth(k,:)',terrainFcn);
end
fig=figure('Visible','off','Color','w','Position',[50,50,1400,1100]);closer=onCleanup(@()close(fig));
tiles=tiledlayout(fig,4,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tiles);hold(ax,'on');plot(ax,h.t,h.truth(:,3),'Color',blue,'LineWidth',1.5);
plot(ax,h.t,h.estimate(:,3),'--','Color',teal);plot(ax,h.t,h.altitude(:,3),':','Color',orange,'LineWidth',1.4);
set(ax,'YDir','reverse');ylabel(ax,'Depth / m');legend(ax,{'Actual','Estimated','Reference'},'Location','eastoutside');grid(ax,'on');
ax=nexttile(tiles);yyaxis(ax,'left');plot(ax,h.t,h.command(:,1),'Color',blue);ylabel(ax,'Thrust / N');
yyaxis(ax,'right');plot(ax,h.t,rad2deg(h.command(:,2:5)),'LineWidth',1);ylabel(ax,'Fin commands / deg');grid(ax,'on');
legend(ax,{'Thrust','Fin 1','Fin 2','Fin 3','Fin 4'},'Location','eastoutside');
ax=nexttile(tiles);hold(ax,'on');plot(ax,h.t,obstacle,'Color',orange,'LineWidth',1.4);plot(ax,h.t,terrain,'Color',blue,'LineWidth',1.2);
yline(ax,z.staticSpec.minimumClearance_m,':','Obstacle requirement','Color',orange);yline(ax,2,':','Terrain requirement','Color',blue);
ylabel(ax,'Body clearance / m');grid(ax,'on');legend(ax,{'Obstacle (sampled)','Terrain (sampled)'},'Location','eastoutside');
ax=nexttile(tiles);yyaxis(ax,'left');plot(ax,h.t,positionError,'Color',teal);ylabel(ax,'Position error / m');
yyaxis(ax,'right');stairs(ax,h.t,double(h.staticActive),'Color',orange);ylim(ax,[-.05,1.05]);ylabel(ax,'Avoidance active');xlabel(ax,'Simulation time / s');grid(ax,'on');
title(tiles,'Execution, actuator commands and sampled clearance / 执行器与净距证据');
subtitle(tiles,'Offline archived samples; use independent raw replay/audit for qualification. Commands are not measured hardware signals.');
files.timeseries=fullfile(outdir,'static_execution_timeseries.png');exportgraphics(fig,files.timeseries,'Resolution',180);
clear closer;
if ~isempty(world.obstacles)
 lower=inf(1,3);upper=-inf(1,3);
 for k=1:numel(world.obstacles)
  if iscell(world.obstacles),o=world.obstacles{k};else,o=world.obstacles(k);end
  if strcmp(o.kind,'sphere'),lo=o.centerNED(:)'-o.radius_m;hi=o.centerNED(:)'+o.radius_m;
  else,lo=[o.centerNE(:)'-o.radius_m,min(o.depthLimits_m)];hi=[o.centerNE(:)'+o.radius_m,max(o.depthLimits_m)];end
  lower=min(lower,lo);upper=max(upper,hi);
 end
 lower=lower-10;upper=upper+10;
 trueLocal=h.truth(:,1:3);estimateLocal=h.estimate(:,1:3);
 inside=all(trueLocal>=lower&trueLocal<=upper,2);trueLocal(~inside,:)=NaN;
 insideEstimate=all(estimateLocal>=lower&estimateLocal<=upper,2);estimateLocal(~insideEstimate,:)=NaN;
 detourLocal=trueLocal;detourLocal(~h.staticActive,:)=NaN;
 fig=figure('Visible','off','Color','w','Position',[50,50,1600,780]);closer=onCleanup(@()close(fig));
 tiles=tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');a3=nexttile(tiles);hold(a3,'on');a2=nexttile(tiles);hold(a2,'on');
 for k=1:numel(world.obstacles)
  if iscell(world.obstacles),o=world.obstacles{k};else,o=world.obstacles(k);end
  if strcmp(o.kind,'sphere')
   surf(a3,o.centerNED(1)+o.radius_m*u,o.centerNED(2)+o.radius_m*v,o.centerNED(3)+o.radius_m*w,'FaceColor',[.72,.20,.19],'FaceAlpha',.42,'EdgeColor','none','HandleVisibility','off');
   angle=linspace(0,2*pi,100);fill(a2,o.centerNED(1)+o.radius_m*cos(angle),o.centerNED(3)+o.radius_m*sin(angle),[.72,.20,.19],'FaceAlpha',.25,'EdgeColor',[.72,.20,.19],'HandleVisibility','off');
  else
   [a,b,c]=cylinder(o.radius_m,40);c=min(o.depthLimits_m)+diff(o.depthLimits_m)*c;
   surf(a3,a+o.centerNE(1),b+o.centerNE(2),c,'FaceColor',[.72,.20,.19],'FaceAlpha',.42,'EdgeColor','none','HandleVisibility','off');
   fill(a2,o.centerNE(1)+o.radius_m*[-1,1,1,-1],o.depthLimits_m([1,1,2,2]),[.72,.20,.19],'FaceAlpha',.25,'EdgeColor',[.72,.20,.19],'HandleVisibility','off');
  end
 end
 plot3(a3,trueLocal(:,1),trueLocal(:,2),trueLocal(:,3),'Color',blue,'LineWidth',2,'DisplayName','真实轨迹');
 plot3(a3,estimateLocal(:,1),estimateLocal(:,2),estimateLocal(:,3),'--','Color',teal,'LineWidth',1.2,'DisplayName','导航估计');
 plot3(a3,detourLocal(:,1),detourLocal(:,2),detourLocal(:,3),'Color',orange,'LineWidth',3,'DisplayName','绕行激活段');
 plot(a2,trueLocal(:,1),trueLocal(:,3),'Color',blue,'LineWidth',2,'DisplayName','真实轨迹');
 plot(a2,estimateLocal(:,1),estimateLocal(:,3),'--','Color',teal,'DisplayName','导航估计');
 plot(a2,detourLocal(:,1),detourLocal(:,3),'Color',orange,'LineWidth',3,'DisplayName','绕行激活段');
 for ax=[a3,a2],set(ax,'FontName','Microsoft YaHei','FontSize',11,'ZDir','reverse');grid(ax,'on');box(ax,'on');axis(ax,'equal');end
 xlim(a3,[lower(1),upper(1)]);ylim(a3,[lower(2),upper(2)]);zlim(a3,[lower(3),upper(3)]);view(a3,38,22);
 xlabel(a3,'北向 N / m');ylabel(a3,'东向 E / m');zlabel(a3,'深度 D / m（向下为正）');title(a3,'局部三维：物理等比例，无垂向夸张');
 set(a2,'YDir','reverse');xlim(a2,[lower(1),upper(1)]);ylim(a2,[lower(3),upper(3)]);xlabel(a2,'北向 N / m');ylabel(a2,'深度 D / m（向下为正）');
 title(a2,'N–深度投影：仅辅助阅读，碰撞以三维审计为准','FontName','Microsoft YaHei');
 legend(a3,'Location','southoutside','Orientation','horizontal');legend(a2,'Location','southoutside','Orientation','horizontal');
 title(tiles,'障碍区局部特写与垂向分离','FontName','Microsoft YaHei');
 subtitle(tiles,'红色障碍均为离线真值；投影忽略东向间距，重叠不等于实际相交。裁剪范围为障碍包络外扩10 m。','FontName','Microsoft YaHei');
 set(findall(fig,'-property','FontName'),'FontName','Microsoft YaHei');
 files.closeup=fullfile(outdir,'static_obstacle_closeup.png');exportgraphics(fig,files.closeup,'Resolution',200);
end
files.scope='Offline truth visualization only. Targets lacking archived depth are displayed at transit depth, not claimed as work-depth truth. Clearance curves are sampled and do not replace the independent physical/map/task audits.';
fid=fopen(fullfile(outdir,'figure_scope.json'),'w');assert(fid>=0);fcloser=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(files,PrettyPrint=true));
end
