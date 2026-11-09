function audit=sta_route_check(route,map,c)
% Map-only tube check: unknown may remain in nominal tail, never in prefix.
sample=sta_segments_sample(route.segments,route.depthSchedule,c.sampleStep_m);
bodyX=c.bodyX_m(:)';halfGap=max(diff(bodyX))/2;
cover=sqrt(c.bodyCrossRadius_m^2+halfGap^2);
attitude=max(abs(bodyX))*2*sin(c.attitudeBudget_rad/2);
geometryRotation=max(abs(bodyX))*c.sampleStep_m/(2*c.turnRadius_m);
navBudget=hypot(c.navRadius95_m,3*c.depthSigma_m);
sampleCover=c.sampleStep_m/2*sqrt(1+max(abs(sample.depthSlope))^2);
margin=cover+navBudget+c.trackingTube_m+c.residualClearance_m+attitude+sampleCover+geometryRotation;
n=size(sample.pointsNED,1);allPoints=zeros(n*numel(bodyX),3);
for k=1:n
 slope=sample.depthSlope(k);unit=[cos(sample.yaw_rad(k)),sin(sample.yaw_rad(k)),slope];unit=unit/norm(unit);
 ids=(k-1)*numel(bodyX)+(1:numel(bodyX));allPoints(ids,:)=sample.pointsNED(k,:)+bodyX'*unit;
end
query=c.queryFcn(map,allPoints,margin);state=reshape(query.state,numel(bodyX),n)';
centerState=zeros(n,1);centerState(any(state<0,2))=-1;centerState(any(state>0,2))=1;
clearance=min(reshape(query.clearance,numel(bodyX),n)',[],2);
firstNonfree=find(centerState~=0,1,'first');
if isempty(firstNonfree),freeDistance=sample.s_m(end);else,freeDistance=max(0,sample.s_m(firstNonfree)-c.sampleStep_m);end
required=c.maximumSpeed_m_s*c.clearHorizon_s;
depthOkay=all(sample.pointsNED(:,3)>=c.depthBounds_m(1)&sample.pointsNED(:,3)<=c.depthBounds_m(2));
slewOkay=all(abs(sample.depthSlope)*c.maximumSpeed_m_s<=c.maximumDepthRate_m_s+1e-10);
audit=struct('accepted',freeDistance>=required&&depthOkay&&slewOkay,...
 'knownOccupiedCollision',any(centerState==1),'freeDistance_m',freeDistance,...
 'requiredClearDistance_m',required,'commitDistance_m',min(c.nominalSpeed_m_s*c.commitHorizon_s,freeDistance),...
 'knownOccupiedSamples',sum(centerState==1),'unknownSamples',sum(centerState<0),...
 'minimumCertifiedClearance_m',min(clearance),'depthOkay',depthOkay,'slewOkay',slewOkay,...
 'sample',sample,'sampleState',centerState,'capsuleCoverRadius_m',cover,...
 'queryMargin_m',margin,'navigationBudget_m',navBudget,'residualClearance_m',c.residualClearance_m,...
 'scope','Conditional sampled tube certificate: body box covered by longitudinal balls, current navigation/tracking/attitude budgets, separate residual clearance and half sample spacing. Sensor-frame budgets are applied by map once; no obstacle truth. Nominal tail unknown is not committed.');
end
