function result=sta_map_query(map,pointsNED,margin)
% Certify each entire requested ball, not just its center or a single ray.
% margin already includes physical section and CURRENT navigation budget.
% Stored CAPTURE-time pose/range bounds are each applied exactly once here.
assert(isnumeric(pointsNED)&&size(pointsNED,2)==3&&all(isfinite(pointsNED),'all'));
n=size(pointsNED,1);if isscalar(margin),margin=repmat(margin,n,1);else,margin=margin(:);end
assert(numel(margin)==n&&all(isfinite(margin))&&all(margin>=0));
state=-ones(n,1,'int8');slack=-inf(n,1);occupiedGap=inf(n,1);
if isempty(pointsNED),result=finish();return;end
if ~isempty(map.prior)
 p=map.prior;boot=p.radius_m-p.positionErrorBound_m-vecnorm(pointsNED-p.centerNED(:)',2,2)-margin;
 good=boot>=0;state(good)=0;slack(good)=boot(good);
end
% Static guard points are conservative exclusions; no motion or stale-time
% expansion. Voxel quantization was added once at insertion, not again here.
if ~isempty(map.occupiedCentersNED)
 reach=max(margin)+max(map.occupiedError_m);lo=min(pointsNED,[],1)-reach;hi=max(pointsNED,[],1)+reach;
 keep=all(map.occupiedCentersNED>=lo&map.occupiedCentersNED<=hi,2);p=map.occupiedCentersNED(keep,:);e=map.occupiedError_m(keep);
 for first=1:256:size(p,1)
  ids=first:min(size(p,1),first+255);distance=sqrt(max(0,sum(pointsNED.^2,2)+sum(p(ids,:).^2,2)'-2*pointsNED*p(ids,:)'));
  occupiedGap=min(occupiedGap,min(distance-e(ids)'-margin,[],2));
 end
end
% Guard exclusions above are global and take precedence over ALL free frames.
% Visit newest observations first; once a full-ball certificate succeeds,
% another frame cannot change its state. Slack is that certificate's lower
% bound, not the maximum slack over every historical observation.
for k=numel(map.frames):-1:1
 pending=find(state~=0&occupiedGap>0);if isempty(pending),break;end
 f=map.frames{k};local=(f.rotation'*(pointsNED(pending,:)'-f.originNED))';d=vecnorm(local,2,2);
 az=atan2(local(:,2),local(:,1));el=atan2(local(:,3),hypot(local(:,1),local(:,2)));
 radius=margin(pending)+f.positionErrorBound_m;alpha=asin(min(1,radius./max(d,eps)))+f.attitudeErrorBound_rad;
 azHalf=asin(min(1,sin(alpha)./max(cos(el),eps)));
 azGap=min(az-f.azimuth_rad(1),f.azimuth_rad(end)-az);elGap=min(el-f.elevation_rad(1),f.elevation_rad(end)-el);
 valid=d>radius&d-radius>=f.minRange_m&azGap>=azHalf&elGap>=alpha;
 if ~any(valid),continue;end
 da=f.azimuth_rad(2)-f.azimuth_rad(1);de=f.elevation_rad(2)-f.elevation_rad(1);
 col=round((az-f.azimuth_rad(1))/da)+1;row=round((el-f.elevation_rad(1))/de)+1;
 half=ceil(max(azHalf/da,alpha/de)+.5);buckets=map.configuration.minimumGridHalfWidths;
 bucket=zeros(numel(pending),1);for j=numel(buckets):-1:1,bucket(half<=buckets(j))=j;end
 valid=valid&bucket>0;
 for j=1:numel(buckets)
  ids=find(valid&bucket==j);if isempty(ids),continue;end
  ind=sub2ind(size(f.prefix_m),row(ids),col(ids));limit=f.minimum{j}(ind);
  azPlane=asin(max(0,min(1,cos(el(ids)).*sin(azGap(ids)))));
  angular=d(ids).*sin(max(0,min(azPlane,elGap(ids))-f.attitudeErrorBound_rad));
  bound=min([limit-d(ids),d(ids)-f.minRange_m,angular],[],2)-radius(ids);
  good=bound>=-1e-9;accepted=pending(ids(good));state(accepted)=0;slack(accepted)=bound(good);
 end
end
blocked=occupiedGap<=0;state(blocked)=1;slack(blocked)=occupiedGap(blocked);
if map.capacityExceeded,state(:)=-1;slack(:)=-Inf;end
result=finish();
 function value=finish()
  value=struct('state',state,'clearance',slack,'clearance_m',slack,'free',state==0,'occupied',state==1,'unknown',state==-1,...
   'occupiedClearance_m',occupiedGap,'frameCount',numel(map.frames),'queryRadius_m',margin,...
   'scope','Entire ball in one observed scan-prefix certificate or explicit launch prior. Newest successful certificate supplies conservative slack (not historical maximum). Global occupied guards override all free certificates. Unknown is blocked. Conditional on declared bounded sensing/navigation assumptions; static occupancy has no time-growing motion radius.');
 end
end
