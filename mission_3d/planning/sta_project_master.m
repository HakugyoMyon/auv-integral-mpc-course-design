function [projected,audit]=sta_project_master(route,xhat,pointer)
% Limited FORWARD reprojection from estimated pose onto original task route.
% Changes only segment/progress in a copy; never changes pose or task ledger.
assert(isstruct(pointer)&&isfield(pointer,'segment')&&isfield(pointer,'progress'));
assert(numel(xhat)>=7&&all(isfinite(xhat(1:7)))&&abs(norm(xhat(4:7))-1)<1e-3);
segments=route.segments;n=numel(segments);assert(n>0&&pointer.segment>=1&&pointer.segment<=n&&pointer.segment==floor(pointer.segment));
assert(isnumeric(pointer.progress)&&isscalar(pointer.progress)&&isfinite(pointer.progress)&&pointer.progress>=-1e-8&&pointer.progress<=segments(pointer.segment).length+1e-8);
window=option(pointer,'projectionWindow_m',100);limit=option(pointer,'projectionMaxSegments',8);
capture=option(pointer,'projectionMaxDistance_m',12);headingLimit=option(pointer,'projectionMaxHeadingError_rad',deg2rad(70));
assert(window>0&&window<=150&&limit>=1&&limit<=16&&limit==floor(limit));
assert(capture>0&&capture<=25&&headingLimit>0&&headingLimit<=pi/2);
edges=[0,cumsum([segments.length])];oldProgress=min(max(pointer.progress,0),segments(pointer.segment).length);
start=edges(pointer.segment)+oldProgress;finish=min(edges(end),start+window);
R=auv6_rotation(xhat(4:7));yaw=atan2(R(2,1),R(1,1));position=xhat(1:2);
projected=pointer;best=Inf;selected=[];candidates={};
for index=pointer.segment:min(n,pointer.segment+limit-1)
 if edges(index)>finish+1e-9,break;end
 low=max(0,start-edges(index));high=min(segments(index).length,finish-edges(index));
 if high<low-1e-9,continue;end
 % Mid-interval angular unwrapping avoids selecting the other lap of a loop.
 [progress,~,~,~,raw]=d11_path_project(position,segments(index),(low+high)/2);
 progress=min(max(progress,low),high);[point,heading]=d11_path_eval(segments(index),progress);point=point(:);
 distance=norm(position-point);delta=abs(atan2(sin(yaw-heading),cos(yaw-heading)));
 globalProgress=edges(index)+progress;advance=globalProgress-start;
 accepted=distance<=capture&&delta<=headingLimit&&advance>=-1e-8;
 score=distance+.5*delta+.001*advance;
 event=struct('segment',index,'progress',progress,'rawProgress',raw,'projectionNE',point,...
  'distance_m',distance,'headingError_rad',delta,'advance_m',advance,'accepted',accepted,'score',score);
 candidates{end+1}=event;
 if accepted&&score<best,best=score;selected=event;end
end
audit=struct('accepted',~isempty(selected),'reason','no_near_forward_heading_consistent_projection',...
 'searchedStart_m',start,'searchedEnd_m',finish,'maximumAdvance_m',window,...
 'maximumSegments',limit,'candidates',{candidates},'distance_m',Inf,'advance_m',0,...
 'headingError_rad',NaN,'projectionNE',nan(2,1),...
 'scope','Estimated-pose finite forward route projection only. Original-ID ledger and all pointer memory fields retained; does not declare work/return complete or certify a connection path.');
if ~isempty(selected)
 projected.segment=selected.segment;projected.progress=selected.progress;
 audit.reason='limited_forward_projection';audit.distance_m=selected.distance_m;
 audit.advance_m=selected.advance_m;audit.headingError_rad=selected.headingError_rad;audit.projectionNE=selected.projectionNE;
end
end
function value=option(pointer,name,default)
if isfield(pointer,name),value=pointer.(name);else,value=default;end
assert(isnumeric(value)&&isscalar(value)&&isfinite(value));
end
