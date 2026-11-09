function audit=sta_trajectory_query(states,map,nav,c,horizon_s)
% Whole estimated nonlinear MPC motion prefix, not geometric centerline only.
if nargin<5,horizon_s=3;end
n=min(size(states,2),floor(horizon_s/.1)+1);states=states(:,1:n);
xs=c.bodyX_m(:)';half=max(diff(xs))/2;radius=sqrt(c.bodyCrossRadius_m^2+half^2);
rotationMargin=max(abs(xs))*2*sin(nav.attitudeErrorBound_rad/2);
clearance=.5;if isfield(c,'residualClearance_m'),clearance=c.residualClearance_m;end
angles=zeros(1,max(0,n-1));
for j=1:n-1
 a=states(4:7,j)/norm(states(4:7,j));b=states(4:7,j+1)/norm(states(4:7,j+1));
 angles(j)=2*acos(min(1,abs(a'*b)));
end
sweepMargin=2*max(abs(xs))*sin(max([0,angles])/4);
margin=radius+nav.positionErrorBound_m+rotationMargin+c.trackingTube_m+clearance+.85*.1/2+sweepMargin;
points=zeros(n*numel(xs),3);
for k=1:n
 R=auv6_rotation(states(4:7,k));ids=(k-1)*numel(xs)+(1:numel(xs));
 points(ids,:)=states(1:3,k)'+xs'*R(:,1)';
end
q=c.queryFcn(map,points,margin);
audit=struct('accepted',all(q.free),'unknown',sum(q.unknown),'occupied',sum(q.occupied),...
 'minimumCertifiedSlack_m',min(q.clearance),'margin_m',margin,'pointsNED',points,...
 'state',q.state,'queryPolicy',func2str(c.queryFcn),'horizon_s',(n-1)*.1,'frameCount',q.frameCount,...
 'scope','Conditional complete-body balls over actual nonlinear MPC predicted prefix with navigation, attitude, tracking, residual and temporal spacing budgets. Not a physical operational safety guarantee.');
end
