function [point,heading,index,progress]=d16_path_preview(plan,index,progress,advance)
% Continue over the real rejoin, rather than freezing at a detour endpoint.
remaining=plan.segments(index).length-progress+sum([plan.segments(index+1:end).length]);
if advance>remaining&&isfield(plan,'resumeRoute')&&isfield(plan,'resumePoint')
 [point,heading,index,progress]=d16_path_preview(plan.resumeRoute,plan.resumePoint.segment,plan.resumePoint.progress,advance-remaining);
else
 [point,heading,index,progress]=d11_path_preview(plan,index,progress,advance);
end
end
