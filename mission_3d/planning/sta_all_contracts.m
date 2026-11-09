function summary=sta_all_contracts(outdir)
% Run actual synthetic contracts, never MIL. Catch/report each real failure.
assert(nargin==1&&(ischar(outdir)||isstring(outdir)));
if ~isfolder(outdir),mkdir(outdir);end
summaryFile=fullfile(outdir,'all_contracts_summary.json');
assert(~isfile(summaryFile),'Use a fresh contract output folder; do not replace evidence.');
names={'perception','planning','projection','anchor','downlooking',...
 'helical','time','lazy','probe','fusion','fusion_world'};
functions={@sta_perception_contract,@sta_planning_contract,...
 @sta_projection_contract,@sta_replan_anchor_contract,@sta_downlooking_contract,...
 @sta_helical_contract,@sta_time_budget_contract,@sta_lazy_contract,...
 @sta_probe_contract,@sta_fusion_contract,@sta_fusion_world_contract};
takesDirectory=[true,true,true,true,false,true,true,true,true,false,true];
expected=[15,6,7,8,8,4,4,2,3,7,3800];
items=cell(size(names));started=datetime('now','TimeZone','UTC');
for k=1:numel(names)
 clock=tic;
 item=struct('name',names{k},'function',func2str(functions{k}),...
  'passed',false,'cases',0,'checks',0,'queries',0,'scenes',0,'expectedCount',expected(k),'countKind','cases',...
  'scope','','duration_s',0,'errorIdentifier','','errorMessage','','errorReport','','result',struct());
 try
  if takesDirectory(k),r=functions{k}(fullfile(outdir,names{k}));else,r=functions{k}();end
  item.result=r;
  assert(isstruct(r)&&isscalar(r)&&isfield(r,'passed')&&isscalar(r.passed)&&logical(r.passed),...
   'Contract did not return explicit scalar passed=true.');
  if isfield(r,'cases'),item.cases=r.cases;actualCount=r.cases;
  elseif isfield(r,'checks'),item.checks=r.checks;item.countKind='checks';actualCount=r.checks;
  elseif isfield(r,'queries'),item.queries=r.queries;item.countKind='queries';actualCount=r.queries;
  else,error('sta:MissingCaseCount','Contract returned no cases/checks/queries count.');end
  if isfield(r,'scenes'),item.scenes=r.scenes;end
  assert(isnumeric(actualCount)&&isscalar(actualCount)&&actualCount==expected(k),...
   'Actual contract count differs from the requested suite; inspect raw result.');
  assert(isfield(r,'scope'),'Contract must state verification scope.');item.scope=r.scope;
  item.passed=true;
 catch err
  item.errorIdentifier=err.identifier;item.errorMessage=err.message;
  item.errorReport=getReport(err,'extended','hyperlinks','off');
  if isstruct(item.result)&&isfield(item.result,'scope'),item.scope=item.result.scope;end
  if isempty(item.scope),item.scope='Failed contract; no success claim.';end
 end
 item.duration_s=toc(clock);items{k}=item;
 % Persist partial evidence after every item, including failures.
 summary=struct('schema','auv.static_contract_suite.v1','passed',false,...
  'startedUTC',char(started),'finishedUTC',char(datetime('now','TimeZone','UTC')),...
  'completed',k,'total',numel(names),'items',{items(1:k)},...
  'scope','Executed synthetic contracts only; no MIL, real-world sensor validation or complete-task claim. Fusion remains experimental even if its finite tests pass.');
 if k==numel(names),summary.passed=all(cellfun(@(a)a.passed,items));end
 fid=fopen(summaryFile,'w');assert(fid>=0);closer=onCleanup(@()fclose(fid));
 fprintf(fid,'%s',jsonencode(summary,PrettyPrint=true));clear closer;
end
disp(summary);
end
