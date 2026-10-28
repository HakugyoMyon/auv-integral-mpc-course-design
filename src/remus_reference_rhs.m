function dz = remus_reference_rhs(t,z,command,P)
% z = [u v w p q r N E D phi theta psi delta_r delta_s RPM]'.
arguments
 t (1,1) double {mustBeFinite}
 z (15,1) double {mustBeFinite}
 command (3,1) double {mustBeFinite}
 P struct
end
x = z(1:12); a = z(13:15);
[dx,~,M] = remus100(x,a,P.current(1),P.current(2),P.current(3));
nuCurrent = [P.current(1)*cos(P.current(2)-x(12)); ...
 P.current(1)*sin(P.current(2)-x(12));P.current(3);0;0;0];
extra = P.force(t);
assert(isequal(size(extra),[6 1]) && all(isfinite(extra)));
if P.kappa ~= 1
 extra = extra+(P.kappa-1)*crossFlowDrag(1.6,.19,.19,x(1:6)-nuCurrent,'cylinder');
end
dx(1:6) = dx(1:6)+M\extra;
dz = [dx;actuator_rhs(a,command,P)];
end
