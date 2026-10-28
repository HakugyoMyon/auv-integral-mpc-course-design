function [reference,memory,info] = los_guidance(x,pathKind,memory,C)
% Uses measured BODY ground velocity and attitude only; no current truth.
[~,R]=eulerang(x(10),x(11),x(12)); velocity=R*x(1:3);
horizontalSpeed=norm(velocity(1:2));
low=horizontalSpeed<C.lowSpeed;
if ~low
 memory.beta=angle(exp(1i*(atan2(velocity(2),velocity(1))-x(12))));
end
[ey,gamma,projection]=path_geometry(x(7:8),pathKind);
target=gamma-atan(ey/C.lookahead)-memory.beta;
increment=(1-exp(-C.Ts/C.psiTau))*angle(exp(1i*(target-memory.psiFiltered)));
increment=min(max(increment,-C.psiRate*C.Ts),C.psiRate*C.Ts);
memory.psiFiltered=memory.psiFiltered+increment;
reference=memory.psiFiltered;
info=struct('ey',ey,'gamma',gamma,'beta',memory.beta, ...
 'groundSpeed',horizontalSpeed,'lowSpeed',low,'projection',projection,'rawHeading',target);
end
