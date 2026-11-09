function y=auv6_euler_state(x)
q=x(4:7)/norm(x(4:7));w=q(1);a=q(2);b=q(3);c=q(4);
phi=atan2(2*(w*a+b*c),1-2*(a*a+b*b));
theta=asin(max(-1,min(1,2*(w*b-c*a))));
psi=atan2(2*(w*c+a*b),1-2*(b*b+c*c));
y=[phi;theta;psi;x(8:18)];
end
