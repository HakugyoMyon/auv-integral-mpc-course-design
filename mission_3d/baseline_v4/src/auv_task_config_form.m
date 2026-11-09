function auv_task_config_form(parent,config,onApply)
% Common operation fields with units; advanced JSON retains the full schema.
config=auv_task_normalize(config);
setappdata(parent,'ConfigurationDirty',false);
g=uigridlayout(parent,[12,2]);g.ColumnWidth={150,'1x'};g.RowHeight={28,28,28,28,28,28,28,28,28,'1x',32,38};g.RowSpacing=5;g.Padding=[8,8,8,8];
t=config.task;
names={'预算 / s','返航 North / m','返航 East / m','航渡深度 / m','返航深度 / m','离底高度 / m','最小转弯半径 / m','随机种子'};
values=[t.maximumTime_s,t.homePose(1:2),t.depth.transit_m,t.depth.return_m,t.depth.workAltitude_m,t.minimumTurnRadius_m,config.execution.seed];
fields=cell(1,numel(values));
for k=1:numel(values)
 a=uilabel(g,'Text',names{k});a.Layout.Row=k;a.Layout.Column=1;
 fields{k}=uieditfield(g,'numeric','Value',values(k),'ValueChangedFcn',@markDirty);fields{k}.Layout.Row=k;fields{k}.Layout.Column=2;
end
extra=[];
switch t.type
 case 'survey'
  columns={'中心N/m','中心E/m','长/m','宽/m','方向/deg','最大线距/m','要求覆盖率'};
  data=[t.survey.centerNE,t.survey.length_m,t.survey.width_m,rad2deg(t.survey.bearing_rad),t.survey.maximumLineSpacing_m,t.survey.requiredCoverage];
  text='巡测区域：滚动表格查看全部7列';
 case 'inspection'
  columns={'North / m','East / m'};data=t.inspection.polylineNE;text='巡检折线点（按行连接）';
  a=uilabel(g,'Text','观测点间距 / m');a.Layout.Row=9;a.Layout.Column=1;
  extra=uieditfield(g,'numeric','Value',t.inspection.pointSpacing_m,'ValueChangedFcn',@markDirty);extra.Layout.Row=9;extra.Layout.Column=2;
 case 'visit'
  columns={'North/m','East/m','深度/m','优先级'};data=[t.visit.pointsNED,t.visit.priority(:)];text='三维点位（优先级越高越先访问）';
 case 'transit'
  columns={'目标North/m','目标East/m','目标深度/m'};data=reshape(t.transit.destinationNED,1,3);text='航渡目标：穿越确认，不是停车或停靠';
 case 'return'
  columns={'固定速度 m/s'};data=.8;text='返航目标使用上方家点及返航深度';
end
if isempty(extra),a=uilabel(g,'Text',text,'FontSize',11);a.Layout.Row=9;a.Layout.Column=[1,2];end
table=uitable(g,'Data',data,'ColumnName',columns,'ColumnEditable',true(1,numel(columns)),'RowName','numbered','CellEditCallback',@markDirty);table.Layout.Row=10;table.Layout.Column=[1,2];
if any(strcmp(t.type,{'visit','inspection'}))
 add=uibutton(g,'Text','增加末行','ButtonPushedFcn',@addRow);add.Layout.Row=11;add.Layout.Column=1;
 remove=uibutton(g,'Text','移除末行','ButtonPushedFcn',@removeRow);remove.Layout.Row=11;remove.Layout.Column=2;
end
button=uibutton(g,'Text','应用表单并检查','ButtonPushedFcn',@applyForm,'FontWeight','bold');button.Layout.Row=12;button.Layout.Column=[1,2];
 function addRow(~,~)
  if size(table.Data,1)<50,table.Data=[table.Data;table.Data(end,:)];markDirty();end
 end
 function removeRow(~,~)
  minimum=1;if strcmp(t.type,'inspection'),minimum=2;end
  if size(table.Data,1)>minimum,table.Data=table.Data(1:end-1,:);markDirty();end
 end
 function applyForm(~,~)
  updated=config;v=cellfun(@(x)x.Value,fields);u=updated.task;
  u.maximumTime_s=v(1);u.homePose(1:2)=v(2:3);u.depth.transit_m=v(4);u.depth.return_m=v(5);u.depth.workAltitude_m=v(6);u.minimumTurnRadius_m=v(7);updated.execution.seed=v(8);
  d=table.Data;
  switch u.type
   case 'survey'
    u.survey.centerNE=d(1,1:2);u.survey.length_m=d(1,3);u.survey.width_m=d(1,4);u.survey.bearing_rad=deg2rad(d(1,5));u.survey.maximumLineSpacing_m=d(1,6);u.survey.requiredCoverage=d(1,7);
   case 'inspection',u.inspection.polylineNE=d;u.inspection.pointSpacing_m=extra.Value;
   case 'visit',u.visit.pointsNED=d(:,1:3);u.visit.priority=d(:,4);
   case 'transit',u.transit.destinationNED=d(1,1:3);
   case 'return' % Speed display is not an additional unverified branch.
    if ~isequal(d,.8),u.speed_m_s=d(1);end
  end
  updated.task=u;setappdata(parent,'ConfigurationDirty',false);onApply(updated);
 end
 function markDirty(~,~)
  setappdata(parent,'ConfigurationDirty',true);
 end
end
