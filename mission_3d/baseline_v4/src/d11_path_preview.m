function [point,heading,index,progress]=d11_path_preview(plan,index,progress,advance)
progress=progress+advance;
while progress>plan.segments(index).length && index<numel(plan.segments)
 progress=progress-plan.segments(index).length;index=index+1;
end
progress=min(max(progress,0),plan.segments(index).length);[p,heading]=d11_path_eval(plan.segments(index),progress);point=p';
end
