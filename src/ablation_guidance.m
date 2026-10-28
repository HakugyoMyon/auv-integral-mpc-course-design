function [reference,memory,info] = ablation_guidance(x,scenario,memory,C)
if ~strcmp(scenario.path,'CIRCLE') || ~isfield(scenario,'radius') || scenario.radius==50
 [reference,memory,info]=los_guidance(x,scenario.path,memory,C);
 return;
end
[~,R]=eulerang(x(10),x(11),x(12)); velocity=R*x(1:3);
horizontalSpeed=norm(velocity(1:2)); low=horizontalSpeed<C.lowSpeed;
if ~low, memory.beta=angle(exp(1i*(atan2(velocity(2),velocity(1))-x(12)))); end
radius=scenario.radius; center=[0;radius]; d=x(7:8)-center; distance=norm(d);
assert(distance>=5,'AUV:Projection','Circle projection too close to center.');
ey=radius-distance; point=center+radius*d/distance; tangent=[-d(2);d(1)]/distance;
gamma=atan2(tangent(2),tangent(1)); target=gamma-atan(ey/C.lookahead)-memory.beta;
increment=(1-exp(-C.Ts/C.psiTau))*angle(exp(1i*(target-memory.psiFiltered)));
increment=min(max(increment,-C.psiRate*C.Ts),C.psiRate*C.Ts);
memory.psiFiltered=memory.psiFiltered+increment; reference=memory.psiFiltered;
info=struct('ey',ey,'gamma',gamma,'beta',memory.beta,'groundSpeed',horizontalSpeed, ...
 'lowSpeed',low,'projection',point,'rawHeading',target);
end
