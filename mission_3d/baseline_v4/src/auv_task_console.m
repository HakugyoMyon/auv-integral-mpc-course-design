function fig=auv_task_console(budgetFile,outputRoot)
% Configuration and execution front end to the SAME verified MIL runner.
% JSON fields retain units/schema; validation is mandatory before every run.
assert(isfile(budgetFile),'Supply the archived mechanical input_budget.json.');
if nargin<2,outputRoot=fullfile(pwd,'task_console_runs');end
assert(ischar(outputRoot)||isstring(outputRoot));
owned=findall(groot,'Type','figure','Tag','AUVTaskConsoleV1');
if ~isempty(owned),fig=owned(1);figure(fig);return;end
catalog=auv_task_catalog();session=[];running=false;lastFile='';track=zeros(0,3);
fig=uifigure('Name','AUV 基础任务工作台 · MPC / MIL','Tag','AUVTaskConsoleV1',...
 'Position',[120,90,1200,800],'Color',[.95,.97,.98],'CloseRequestFcn',@closeConsole);
g=uigridlayout(fig,[4,2]);g.RowHeight={46,'1x',100,70};g.ColumnWidth={450,'1x'};
g.Padding=[16,12,16,12];g.RowSpacing=10;g.ColumnSpacing=14;
select=uidropdown(g,'Items',{'区域巡测','沿线巡检','三维点位访问','航渡至指定点','返航至家点'},...
 'ItemsData',1:5,'Value',3,'ValueChangedFcn',@choose);select.Layout.Row=1;select.Layout.Column=1;
label=uilabel(g,'Text','无障碍 / 0.8 m/s / 转弯半径 ≥25 m / 仅多变量 MPC','FontWeight','bold');label.Layout.Row=1;label.Layout.Column=2;
tabs=uitabgroup(g);tabs.Layout.Row=2;tabs.Layout.Column=1;
basicTab=uitab(tabs,'Title','作业参数');advancedTab=uitab(tabs,'Title','完整 JSON（高级）');
eg=uigridlayout(advancedTab,[1,1]);editor=uitextarea(eg,'FontName','Consolas','FontSize',12,'ValueChangedFcn',@syncForm);
axesPanel=uipanel(g,'Title','名义路线与导航估计（非隐含真值）');axesPanel.Layout.Row=2;axesPanel.Layout.Column=2;
ag=uigridlayout(axesPanel,[1,1]);ax=uiaxes(ag);grid(ax,'on');view(ax,3);xlabel(ax,'East / m');ylabel(ax,'North / m');zlabel(ax,'Depth / m (4x)');ax.ZDir='reverse';daspect(ax,[1,1,.25]);
buttons=uigridlayout(g,[2,4]);buttons.Layout.Row=3;buttons.Layout.Column=[1,2];
buttons.RowHeight={'1x','1x'};
preview=uibutton(buttons,'Text','检查并预览','ButtonPushedFcn',@previewTask);
saveButton=uibutton(buttons,'Text','保存配置','ButtonPushedFcn',@saveTask);
loadButton=uibutton(buttons,'Text','加载配置','ButtonPushedFcn',@loadTask);
startButton=uibutton(buttons,'Text','运行 MIL','ButtonPushedFcn',@startTask,'FontWeight','bold');
pauseButton=uibutton(buttons,'Text','暂停 / 恢复仿真','Enable','off','ButtonPushedFcn',@pauseTask);
returnButton=uibutton(buttons,'Text','请求返航','Enable','off','ButtonPushedFcn',@returnTask);
stopButton=uibutton(buttons,'Text','中止仿真','Enable','off','ButtonPushedFcn',@stopTask);
exportButton=uibutton(buttons,'Text','导出结果','Enable','off','ButtonPushedFcn',@exportTask);
status=uitextarea(g,'Editable','off','FontSize',12);status.Layout.Row=4;status.Layout.Column=[1,2];
status.Value={'编辑任务 JSON（NED：北/东/深度，m；角度 rad）。每次运行重新预检。';'暂停仅冻结仿真时间；中止不是物理安全停车；成功判定仍需原始数据审计。'};
choose();
 function choose(~,~)
  editor.Value=splitlines(string(jsonencode(catalog{select.Value},PrettyPrint=true)));syncForm();
  cla(ax);lastFile='';exportButton.Enable='off';
 end
 function syncForm(~,~)
  try
   config=jsondecode(strjoin(string(editor.Value),newline));
   taskTypes={'survey','inspection','visit','transit','return'};
   index=find(strcmp(config.task.type,taskTypes));assert(isscalar(index),'Unknown task type.');select.Value=index;
   delete(basicTab.Children);auv_task_config_form(basicTab,config,@applyForm);
  catch ex,showError(ex);end
 end
 function applyForm(config)
  editor.Value=splitlines(string(jsonencode(config,PrettyPrint=true)));previewTask();
 end
 function config=readConfig()
  assert(~getappdata(basicTab,'ConfigurationDirty'),'表单有未应用的修改，请先点击“应用表单并检查”。');
  config=jsondecode(strjoin(string(editor.Value),newline));
  [check,~]=auv_task_validate(config);assert(check.valid,strjoin(check.errors,newline));
 end
 function showPlan(plan)
  % XY geometry at a labelled display plane avoids invented height profiles.
  cla(ax);hold(ax,'on');
  for k=1:numel(plan.segments)
   seg=plan.segments(k);xy=sampleSegment(seg);
   plot3(ax,xy(:,2),xy(:,1),zeros(size(xy,1),1),'Color',[.22,.55,.65],'LineWidth',1.2);
  end
  if isfield(plan.targets,'positionsNED')
   pts=plan.targets.positionsNED;scatter3(ax,pts(:,2),pts(:,1),pts(:,3),40,[.9,.45,.12],'filled');
  end
  [goal,~]=auv_task_terminal(plan);scatter3(ax,goal(2),goal(1),goal(3),65,[.2,.4,.8],'filled');
  title(ax,'平面路线绘于 depth=0 参考面；点位显示指定深度');hold(ax,'off');
 end
 function xy=sampleSegment(seg)
  q=linspace(0,seg.length,max(2,ceil(seg.length/2)+1));xy=zeros(numel(q),2);
  for ii=1:numel(q)
   if strcmp(seg.kind,'line'),xy(ii,:)=seg.start'+q(ii)*[cos(seg.heading0),sin(seg.heading0)];
   else
    curvature=seg.direction/seg.radius;
    theta=seg.heading0+curvature*q(ii);xy(ii,:)=seg.start'+[(sin(theta)-sin(seg.heading0))/curvature,(-cos(theta)+cos(seg.heading0))/curvature];
   end
  end
 end
 function previewTask(~,~)
  try
   config=readConfig();[~,plan]=auv_task_validate(config);showPlan(plan);
   status.Value={sprintf('预检通过：%s；名义长度 %.1f m，名义 ETA %.1f s。',config.task.type,plan.totalLength_m,plan.geometricETA_s);'仅候选几何可行；不是动力学、能量预算或完成任务的保证。'};
  catch ex,showError(ex);end
 end
 function saveTask(~,~)
  try
   config=readConfig();[name,folder]=uiputfile('*.json','保存到新配置文件');if isequal(name,0),return;end
   auv_task_save(config,fullfile(folder,name));status.Value={'配置已保存。'};
  catch ex,showError(ex);end
 end
 function loadTask(~,~)
  try
   [name,folder]=uigetfile('*.json','加载任务配置');if isequal(name,0),return;end
   config=auv_task_load(fullfile(folder,name));editor.Value=splitlines(string(jsonencode(config,PrettyPrint=true)));syncForm();previewTask();
  catch ex,showError(ex);end
 end
 function startTask(~,~)
  if running,return;end
  try
   config=readConfig();[~,plan]=auv_task_validate(config);showPlan(plan);
   out=fullfile(outputRoot,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));assert(~isfolder(out));
   running=true;setRunning(true);track=zeros(0,3);session=AUVTaskSession();session.StatusCallback=@updateStatus;
   cleanup=onCleanup(@()setRunning(false));
   report=auv_task_run(config,budgetFile,out,config.task.maximumTime_s,session);
   lastFile=fullfile(out,'mil.mat');exportButton.Enable='on';
   status.Value={sprintf('结束：原任务通过=%d，人工返航完成=%d，原因=%s',report.physical_and_task_pass,report.operator_return_complete,report.abort);['原始文件：',lastFile]};
  catch ex,showError(ex);end
 end
 function setRunning(value)
  running=value;if ~isvalid(fig),return;end
  if value,ed='off';run='on';else,ed='on';run='off';end
  select.Enable=ed;editor.Editable=ed;preview.Enable=ed;saveButton.Enable=ed;loadButton.Enable=ed;startButton.Enable=ed;
  inputs=findall(basicTab,'-property','Enable');for ii=1:numel(inputs),inputs(ii).Enable=ed;end
  pauseButton.Enable=run;returnButton.Enable=run;stopButton.Enable=run;
 end
 function updateStatus(s)
  if ~isvalid(fig)||~isfield(s,'time_s'),return;end
  if mod(round(s.time_s*10),10)~=0,return;end
  track(end+1,:)=s.estimatedNED';
  line=findobj(ax,'Tag','EstimatedTrack');
  if isempty(line),hold(ax,'on');plot3(ax,track(:,2),track(:,1),track(:,3),'Color',[.03,.35,.45],'LineWidth',2,'Tag','EstimatedTrack');hold(ax,'off');
  else,line.XData=track(:,2);line.YData=track(:,1);line.ZData=track(:,3);end
  status.Value={sprintf('仿真 %.1f s；确认 %d/%d；人工返航=%d；%s',s.time_s,s.confirmed,s.total,s.operatorReturn,s.abort);'轨迹为导航估计；关闭窗口前请先中止或等待航次结束。'};
 end
 function pauseTask(~,~)
  if isempty(session),return;end
  if session.Paused,session.request('resume_sim');status.Value={'恢复仿真时间。'};
  else,session.request('pause_sim');status.Value={'已请求暂停仿真时间；不代表实体 AUV 悬停。'};end
 end
 function returnTask(~,~)
  if ~isempty(session),session.request('return_home');status.Value={'返航已请求，等待当前状态规划和 MPC 预检；不会把未完成作业改为完成。'};end
 end
 function stopTask(~,~)
  if ~isempty(session),session.request('stop_sim');status.Value={'已请求中止 MIL；不代表实体 AUV 安全停止。'};end
 end
 function exportTask(~,~)
  try
   assert(~running&&isfile(lastFile));out=[fileparts(lastFile),'_export_',char(datetime('now','Format','HHmmss_SSS'))];
   auv_task_export(lastFile,out);status.Value={['结果已导出：',out];'失败/中止结果同样保留，不会被标记为任务成功。'};
  catch ex,showError(ex);end
 end
 function showError(ex)
  if isvalid(fig),status.Value={['错误：',ex.message]};end
 end
 function closeConsole(~,~)
  if running,status.Value={'航次仍在运行。请先中止仿真或等待结束，再关闭此工作台。'};
  else,delete(fig);end
 end
end
