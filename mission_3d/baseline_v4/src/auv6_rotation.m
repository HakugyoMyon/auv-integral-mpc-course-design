function R=auv6_rotation(q)
q=q/norm(q);e=q(2:4);S=auv6_skew(e);R=eye(3)+2*q(1)*S+2*S*S;
end
