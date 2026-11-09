function [dx, aux] = auv6_rhs(x, pwm, tauOther, p)
% State [position_NED; quaternion_scalar_first; velocity_FRD; thrust_N].
% tauOther is the wrench at the declared body origin, in [N; N m].
q = x(4:7);
qUnit = q/norm(q);
s = qUnit(1);
e = qUnit(2:4);
R = eye(3) + 2*s*auv6_skew(e) + 2*auv6_skew(e)^2;
nu = x(8:13);
v = nu(1:3);
omega = nu(4:6);

% Momentum form of C(nu)*nu, valid for the constant inertia in still water.
momentum = p.M*nu;
convective = [cross(omega,momentum(1:3)); ...
    cross(v,momentum(1:3))+cross(omega,momentum(4:6))];
drag = (p.d1+p.d2.*abs(nu)).*nu;
down = R'*[0;0;1];
gravity = p.W*down;
buoyancy = -p.B*down;
hydrostatic = [gravity+buoyancy; cross(p.rG,gravity)+cross(p.rB,buoyancy)];
pwm = min(max(pwm,p.pwm(1)),p.pwm(end));
target = interp1(p.pwm,p.staticThrust,pwm,'linear');
if abs(pwm-1500)<=25
    target = 0;
end
thrust = [x(14);0;0];
tauT = [thrust;cross(p.thrusterPosition,thrust)];
nuDot = p.M\(hydrostatic + tauT + tauOther(:) - convective - drag);
qDot = .5*[-q(2:4)'*omega; q(1)*omega + cross(q(2:4),omega)];
dx = [R*v; qDot; nuDot; (target-x(14))/p.thrusterLag_s];
aux = struct('R',R,'hydrostatic',hydrostatic,'convective',convective, ...
    'drag',drag,'targetThrust_N',target,'thrusterWrench',tauT);
end
