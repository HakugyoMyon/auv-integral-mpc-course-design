classdef AUVTaskSession < handle
 % One in-process command queue. Pause stops simulation time, not the AUV.
 properties (SetAccess=private)
  Paused=false
  StopRequested=false
  State='idle'
  LastStatus=struct()
  Log={}
 end
 properties
  StatusCallback=[]
 end
 properties (Access=private)
  Queue={}
  Scheduled={}
  NextId=1
 end
 methods
  function id=request(obj,kind)
   allowed={'pause_sim','resume_sim','return_home','stop_sim'};
   assert(any(strcmp(kind,allowed)),'Unknown operator command.');
   id=obj.NextId;obj.NextId=id+1;
   obj.Queue{end+1}=struct('id',id,'kind',kind,'source','operator');
  end
  function schedule(obj,time,kind)
   assert(isscalar(time)&&isfinite(time)&&time>=0);
   assert(any(strcmp(kind,{'pause_sim','resume_sim','return_home','stop_sim'})));
   assert(~any(strcmp(kind,{'pause_sim','resume_sim'})),...
    'Simulation-time scheduled pause cannot resume itself: use live callbacks.');
   obj.Scheduled{end+1}=struct('time',time,'kind',kind);
  end
  function commands=poll(obj,time)
   commands={};
   due=find(cellfun(@(x)x.time<=time+1e-9,obj.Scheduled));
   if ~isempty(due)
    [~,order]=sort(cellfun(@(x)x.time,obj.Scheduled(due)));
    for k=due(order)
     e=obj.Scheduled{k};obj.request(e.kind);obj.Queue{end}.source='scripted_sim_time';obj.Queue{end}.requestedAt=e.time;
    end
    obj.Scheduled(due)=[];
   end
   if numel(obj.Queue)>1
    [~,order]=sort(cellfun(@(x)x.id,obj.Queue));obj.Queue=obj.Queue(order);
   end
   while ~isempty(obj.Queue)
    e=obj.Queue{1};obj.Queue(1)=[];e.receivedAt=time;
    switch e.kind
     case 'pause_sim',obj.Paused=true;
     case 'resume_sim',obj.Paused=false;
     case 'stop_sim',obj.StopRequested=true;obj.Paused=false;
    end
    commands{end+1}=e;obj.Log{end+1}=e;
   end
   if obj.StopRequested,obj.State='stopping_simulation';
   elseif obj.Paused,obj.State='paused_simulation';else,obj.State='running';end
  end
  function acknowledge(obj,command,time,accepted,reason)
   obj.Log{end+1}=struct('id',command.id,'kind','decision',...
    'command',command.kind,'time',time,'accepted',logical(accepted),'reason',reason);
  end
  function publish(obj,status)
   obj.LastStatus=status;
   if ~isempty(obj.StatusCallback),obj.StatusCallback(status);end
  end
  function finish(obj,report)
   obj.State='ended';obj.LastStatus=report;
   if ~isempty(obj.StatusCallback),obj.StatusCallback(report);end
  end
 end
end
