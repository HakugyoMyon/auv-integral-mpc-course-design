function x=auv6_from_euler_state(y)
ph=y(1)/2;th=y(2)/2;ps=y(3)/2;
cr=cos(ph);sr=sin(ph);ct=cos(th);st=sin(th);cy=cos(ps);sy=sin(ps);
x=zeros(18,1);x(3)=10;
x(4:7)=[cr*ct*cy+sr*st*sy;sr*ct*cy-cr*st*sy;cr*st*cy+sr*ct*sy;cr*ct*sy-sr*st*cy];
x(8:18)=y(4:14);
end
