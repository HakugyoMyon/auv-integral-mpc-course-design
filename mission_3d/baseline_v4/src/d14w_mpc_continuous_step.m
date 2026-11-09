function [u,memory,info]=d14w_mpc_continuous_step(z,reference,inputReference,m,memory,forceFailure)
if nargin<6,forceFailure=false;end
if ~isempty(memory)
 previous=m.z0(4)+memory.previousX(4);
 z(4)=previous+atan2(sin(z(4)-previous),cos(z(4)-previous));
end
headings=unwrap([z(4),reference(4,:)]);reference(4,:)=headings(2:end);
[u,memory,info]=d14w_mpc_step(z,reference,inputReference,m,memory,forceFailure);
info.heading_error_used_rad=reference(4,1)-z(4);
info.continuous_heading_rad=z(4);
end
