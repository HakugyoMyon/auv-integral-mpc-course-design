function active=auv_task_window_active(execution,field,time)
% Explicit simulation fault windows [start inclusive, end exclusive], seconds.
active=false;if ~isfield(execution,field),return;end
w=execution.(field);if isempty(w),return;end
active=any(time>=w(:,1)&time<w(:,2));
end
