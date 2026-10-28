function value = auv_sl_memory(action,input)
% Stable 13-double representation of existing MATLAB controller memory.
if strcmp(action,'pack')
 m=input;fail=0;eta=0;last=NaN;
 if isfield(m,'mpcConsecutiveFailures'),fail=m.mpcConsecutiveFailures;end
 if isfield(m,'ablationIntegral'),eta=m.ablationIntegral;end
 if isfield(m,'lastAblationReference'),last=m.lastAblationReference;end
 value=[m.integral;m.rateFiltered;m.psiFiltered;m.beta;m.previousCommand;fail;eta;last];
else
 a=input;value=struct('integral',a(1:3),'rateFiltered',a(4:5),'psiFiltered',a(6), ...
 'beta',a(7),'previousCommand',a(8:10),'mpcConsecutiveFailures',a(11),'ablationIntegral',a(12));
 if ~isnan(a(13)),value.lastAblationReference=a(13);end
end
end
