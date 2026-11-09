function valid=d12_payload_truth_check(indices,centers,cellRadius,x,c,terrain,cellBearing)
% OFFLINE audit only. Never call from navigation, guidance or task decisions.
if nargin<7,cellBearing=0;end
T=[cos(cellBearing),-sin(cellBearing);sin(cellBearing),cos(cellBearing)];
R=auv6_rotation(x(4:7));origin=x(1:3)+R*c.mount;valid=false(size(indices));
for j=1:numel(indices)
 point=centers(indices(j),:);okay=true;
 % cellRadius is the circumradius used in the projection erosion.
 halfWidth=cellRadius/sqrt(2);
 for dn=[-halfWidth,halfWidth]
  for de=[-halfWidth,halfWidth]
   ne=point+(T*[dn;de])';ground=[ne';terrain(ne(1),ne(2))];v=R'*(ground-origin);range=norm(v);
   inside=v(3)>0&&abs(v(1))<=v(3)*tan(c.alongHalfAngle_rad)&&abs(v(2))<=v(3)*tan(c.crossHalfAngle_rad)&&range<=c.maximumRange_m;
   if inside
    for f=[.25,.5,.75,.95]
     p=origin+f*(ground-origin);if p(3)>=terrain(p(1),p(2)),inside=false;break;end
    end
   end
   okay=okay&&inside;
  end
 end
 valid(j)=okay;
end
end
