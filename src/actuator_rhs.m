function da = actuator_rhs(a,command,P)
arguments
 a (3,1) double {mustBeFinite}
 command (3,1) double {mustBeFinite}
 P struct
end
target = min(max(command,P.lower),P.upper);
da = min(max((target-a)./P.tau,-P.rate),P.rate);
da(a<=P.lower & da<0) = 0;
da(a>=P.upper & da>0) = 0;
end
