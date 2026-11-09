function [dx,aux]=auv6_actuated_rhs(x,command,p,f)
% 18 states: original14 plus actual upper/lower/port/starboard fin angles.
% command=[PWM_us; four commanded hydrodynamic fin angles_rad]. Still water.
assert(numel(x)==18 && numel(command)==5);
target=min(max(command(2:5),-f.angleLimit),f.angleLimit);
deltaDot=min(max((target-x(15:18))/f.lag,-f.rateLimit),f.rateLimit);
[tau,fin]=auv6_fin_loads(x(8:13),x(15:18),f);
[body,aux]=auv6_rhs(x(1:14),command(1),tau,p);
dx=[body;deltaDot];aux.fin=fin;aux.finWrench=tau;
aux.deltaTarget=target;aux.deltaRate=deltaDot;
end
