function paths=d15_dubins_candidates(startPose,goalPose,radius)
% MATLAB adaptation of Andrew Walker's six Dubins words, MIT licensed.
% Upstream commit: 2406a2281dd3b63712f8cd634b65cdcb1c540f33
% https://github.com/AndrewWalker/Dubins-Curves
% Copyright (c) 2008-2018, Andrew Walker
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.
assert(radius>0&&numel(startPose)==3&&numel(goalPose)==3&&all(isfinite([startPose(:);goalPose(:)])));
delta=goalPose(1:2)-startPose(1:2);d=norm(delta)/radius;theta=0;if d>0,theta=atan2(delta(2),delta(1));end
a=wrap(startPose(3)-theta);b=wrap(goalPose(3)-theta);sa=sin(a);sb=sin(b);ca=cos(a);cb=cos(b);cab=cos(a-b);
names={'LSL','LSR','RSL','RSR','RLR','LRL'};values=nan(6,3);
p2=2+d*d-2*cab+2*d*(sa-sb);
if p2>=-1e-12,tmp=atan2(cb-ca,d+sa-sb);values(1,:)=[wrap(tmp-a),sqrt(max(0,p2)),wrap(b-tmp)];end
p2=-2+d*d+2*cab+2*d*(sa+sb);
if p2>=-1e-12,p=sqrt(max(0,p2));tmp=atan2(-ca-cb,d+sa+sb)-atan2(-2,p);values(2,:)=[wrap(tmp-a),p,wrap(tmp-b)];end
p2=-2+d*d+2*cab-2*d*(sa+sb);
if p2>=-1e-12,p=sqrt(max(0,p2));tmp=atan2(ca+cb,d-sa-sb)-atan2(2,p);values(3,:)=[wrap(a-tmp),p,wrap(b-tmp)];end
p2=2+d*d-2*cab+2*d*(sb-sa);
if p2>=-1e-12,tmp=atan2(ca-cb,d-sa+sb);values(4,:)=[wrap(a-tmp),sqrt(max(0,p2)),wrap(tmp-b)];end
tmp=(6-d*d+2*cab+2*d*(sa-sb))/8;
if abs(tmp)<=1+1e-12
 p=wrap(2*pi-acos(min(max(tmp,-1),1)));phi=atan2(ca-cb,d-sa+sb);t=wrap(a-phi+p/2);
 values(5,:)=[t,p,wrap(a-b-t+p)];
end
tmp=(6-d*d+2*cab+2*d*(sb-sa))/8;
if abs(tmp)<=1+1e-12
 p=wrap(2*pi-acos(min(max(tmp,-1),1)));phi=atan2(ca-cb,d+sa-sb);t=wrap(-a-phi+p/2);
 values(6,:)=[t,p,wrap(b-a-t+p)];
end
paths=struct('word',{},'lengths',{},'length_m',{});
if d<1e-12&&abs(atan2(sin(a-b),cos(a-b)))<1e-12
 paths(1)=struct('word','S','lengths',0,'length_m',0);
end
for k=1:6
 if all(isfinite(values(k,:))),paths(end+1)=struct('word',names{k},'lengths',radius*values(k,:),'length_m',radius*sum(values(k,:)));end
end
[~,order]=sort([paths.length_m]);paths=paths(order);
end
function value=wrap(value)
value=mod(value,2*pi);if abs(value-2*pi)<1e-12,value=0;end
end
