function result=d72_fresh_gate(context,reference,inputReference,plannedSpeed,m,p,f,step)
% Preserve D14W limits; reject predicted departure from its same speed domain.
if nargin<8,step=.05;end
result=d14w_fresh_gate(context,reference,inputReference,plannedSpeed,m,p,f,step);
result.predicted_horizontal_speed_min_m_s=NaN;
result.predicted_horizontal_speed_max_m_s=NaN;
if ~isfield(result,'predictedTrajectory'),return;end
x=result.predictedTrajectory;speed=zeros(1,size(x,2));
for k=1:numel(speed)
 R=auv6_rotation(x(4:7,k));speed(k)=norm(R(1:2,:)*x(8:10,k));
end
result.predicted_horizontal_speed_min_m_s=min(speed);
result.predicted_horizontal_speed_max_m_s=max(speed);
result.predicted_speed_domain_pass=all(abs(speed-plannedSpeed)<=.05+1e-12);
if result.accepted&&~result.predicted_speed_domain_pass
 result.accepted=false;result.reason='predicted_speed_outside_candidate_domain';
end
end
