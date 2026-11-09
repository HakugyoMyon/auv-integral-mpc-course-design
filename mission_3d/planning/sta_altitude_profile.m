function [z,slope,schedule,meta]=sta_altitude_profile(pointsNE,s_m,xhat,c)
% Local measured-plane NOMINAL prediction, never a terrain safety guarantee.
s=s_m(:);assert(size(pointsNE,1)==numel(s)&&size(pointsNE,2)==2&&s(1)==0&&all(diff(s)>0));
usable=isfield(c,'terrainPlane')&&isfield(c,'currentTime_s')&&...
 isfield(c.terrainPlane,'valid')&&c.terrainPlane.valid&&...
 isfield(c.terrainPlane,'time')&&isfield(c.terrainPlane,'gradientNE')&&...
 c.currentTime_s>=c.terrainPlane.time&&c.currentTime_s-c.terrainPlane.time<=.7;
gradient=[0;0];reason='missing_invalid_or_stale_measured_plane';
if usable
 gradient=c.terrainPlane.gradientNE(:);usable=numel(gradient)==2&&all(isfinite(gradient));
end
if usable,reason='fresh_local_plane_nominal_prediction';else,gradient=[0;0];end
desired=c.nominalDepth_m+(pointsNE-xhat(1:2)')*gradient;
z=zeros(size(s));z(1)=c.referenceDepth_m;rate=c.maximumDepthRate_m_s/c.maximumSpeed_m_s;
for k=2:numel(s),change=rate*(s(k)-s(k-1));z(k)=z(k-1)+min(max(desired(k)-z(k-1),-change),change);end
slopes=diff(z)./diff(s);slope=[slopes;slopes(end)];schedule=[s,z];
within=all(z>=c.depthBounds_m(1)&z<=c.depthBounds_m(2));
meta=struct('planeUsed',usable,'reason',reason,'gradientNE',gradient,...
 'withinDepthBounds',within,'bounds',c.depthBounds_m,'accepted',within,...
 'executionVariant','altitude_plane_nominal_v2',...
 'scope','Fresh four-beam local-plane extrapolation and reference slew screen only. No truth input, global terrain model, future-observation guarantee or whole-tail physical safety certificate. Actual MPC/map prefix admission remains mandatory.');
end
