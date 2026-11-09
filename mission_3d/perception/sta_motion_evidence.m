function files=sta_motion_evidence(file,outdir)
% Offline motion/actuation evidence. No truth enters controller or planner.
if nargin<2,outdir=fullfile(fileparts(file),'motion_evidence');end
if ~isfolder(outdir),mkdir(outdir);end
z=load(file,'h','plan','staticSpec');h=z.h;n=numel(h.t);task=z.plan.task;
trueHeading=zeros(n,1);estimatedHeading=trueHeading;groundSpeed=trueHeading;estimatedGroundSpeed=trueHeading;
for k=1:n
 xt=h.truth(k,:)';xe=h.estimate(k,:)';a=auv6_euler_state(xt);b=auv6_euler_state(xe);
 trueHeading(k)=a(3);estimatedHeading(k)=b(3);
 velocity=auv6_rotation(xt(4:7))*xt(8:10);groundSpeed(k)=norm(velocity(1:2));
 velocity=auv6_rotation(xe(4:7))*xe(8:10);estimatedGroundSpeed(k)=norm(velocity(1:2));
end
desiredHeading=unwrap(h.reference(:,4));trueHeading=unwrap(trueHeading);estimatedHeading=unwrap(estimatedHeading);
trueHeading=trueHeading+2*pi*round((desiredHeading(1)-trueHeading(1))/(2*pi));
estimatedHeading=estimatedHeading+2*pi*round((desiredHeading(1)-estimatedHeading(1))/(2*pi));
desiredSpeed=task.speed_m_s*ones(n,1);desiredDepth=h.reference(:,1);
heightMode=logical(h.altitude(:,1));actualAltitude=nan(n,1);estimatedAltitude=h.altitude(:,2);desiredAltitude=nan(n,1);
hasAltitude=isfield(task.depth,'workAltitude_m')&&any(heightMode);
if isfield(z.staticSpec.world,'terrainFcn'),terrain=z.staticSpec.world.terrainFcn;
elseif isfield(z.staticSpec.world,'bottomDepth_m'),bottom=z.staticSpec.world.bottomDepth_m;terrain=@(a,b)bottom+zeros(size(a+b));
else,error('Missing archived terrain model.');end
for k=1:n,actualAltitude(k)=terrain(h.truth(k,1),h.truth(k,2))-h.truth(k,3);end
if hasAltitude,desiredAltitude(heightMode)=task.depth.workAltitude_m;end
tab=table(h.t,rad2deg(trueHeading),rad2deg(estimatedHeading),rad2deg(desiredHeading),groundSpeed,estimatedGroundSpeed,...
 h.truth(:,8),h.estimate(:,8),desiredSpeed,h.truth(:,3),h.estimate(:,3),desiredDepth,actualAltitude,estimatedAltitude,desiredAltitude,heightMode,...
 h.command(:,1),rad2deg(h.command(:,2)),rad2deg(h.command(:,3)),rad2deg(h.command(:,4)),rad2deg(h.command(:,5)),...
 'VariableNames',{'time_s','true_heading_unwrapped_deg','estimated_heading_unwrapped_deg','desired_heading_unwrapped_deg',...
 'true_horizontal_ground_speed_m_s','estimated_horizontal_ground_speed_m_s','true_body_surge_u_m_s','estimated_body_surge_u_m_s',...
 'desired_speed_m_s','true_depth_m','estimated_depth_m','desired_depth_m','true_altitude_center_m','estimated_altitude_m','desired_work_altitude_m','height_work_mode',...
 'command_thrust_N','command_fin1_deg','command_fin2_deg','command_fin3_deg','command_fin4_deg'});
files.csv=fullfile(outdir,'motion_control.csv');writetable(tab,files.csv);
fig=figure('Visible','off','Color','w','Position',[50,50,1500,1050]);closer=onCleanup(@()close(fig));
tiles=tiledlayout(fig,3+hasAltitude,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tiles);plot(ax,h.t,rad2deg(trueHeading),h.t,rad2deg(estimatedHeading),'--',h.t,rad2deg(desiredHeading),':','LineWidth',1.3);
ylabel(ax,'航向 / deg');legend(ax,{'真实航向','估计航向','期望航向'},'Location','eastoutside');grid(ax,'on');
ax=nexttile(tiles);plot(ax,h.t,groundSpeed,h.t,h.truth(:,8),'--',h.t,desiredSpeed,':','LineWidth',1.3);
ylabel(ax,'速度 / (m/s)');legend(ax,{'水平对地速度','艇体系纵向速度 u','期望速度'},'Location','eastoutside');grid(ax,'on');
ax=nexttile(tiles);plot(ax,h.t,h.truth(:,3),h.t,h.estimate(:,3),'--',h.t,desiredDepth,':','LineWidth',1.3);
set(ax,'YDir','reverse');ylabel(ax,'深度 / m（向下）');legend(ax,{'真实深度','估计深度','期望深度'},'Location','eastoutside');grid(ax,'on');
if hasAltitude
 ax=nexttile(tiles);plot(ax,h.t,actualAltitude,h.t,estimatedAltitude,'--',h.t,desiredAltitude,':','LineWidth',1.3);
 ylabel(ax,'离底高度 / m');legend(ax,{'真实中心离底高度','测量／导航估计高度','仅作业段期望高度'},'Location','eastoutside');grid(ax,'on');
end
xlabel(ax,'仿真时间 / s');title(tiles,'MPC航向、速度、深度与作业高度控制证据');
subtitle(tiles,{'真实状态仅用于离线展示；水平对地速度与艇体系 u 不同。控制对象为有执行器动力学的六自由度航行器。',...
 '任务为路径跟随，并非严格时间标定的轨迹跟踪；返航段不自动要求保持作业离底高度。执行器指令详见CSV及原执行曲线。'});
set(findall(fig,'-property','FontName'),'FontName','Microsoft YaHei');set(findall(fig,'Type','axes'),'FontSize',11);
files.figure=fullfile(outdir,'motion_control_evidence.png');exportgraphics(fig,files.figure,'Resolution',180);
files.scope='Offline archived truth/estimate/reference motion evidence and archived thrust/fin command CSV. Body surge u and horizontal NED ground speed are distinct. Altitude truth uses terrain at body center, not full-body clearance. Desired work altitude is shown only where logged height-work mode is active; path following has no strict time-indexed arrival requirement. Commands are simulated actuator inputs, not hardware measurements.';
fid=fopen(fullfile(outdir,'motion_evidence_scope.json'),'w');assert(fid>=0);fcloser=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(files,PrettyPrint=true));
end
