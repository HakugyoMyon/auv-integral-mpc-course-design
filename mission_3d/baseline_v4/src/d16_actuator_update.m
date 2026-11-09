function next=d16_actuator_update(act,command,p,f,dt)
% Exact held-command solution of the SAME first-order/rate-limited driver.
% Does not read the plant, add a sensor or alter physical actuator equations.
assert(numel(act)==5&&numel(command)==5&&dt>0);
pwm=min(max(command(1),p.pwm(1)),p.pwm(end));target=interp1(p.pwm,p.staticThrust,pwm,'linear');
if abs(pwm-1500)<=25,target=0;end
next=act;next(1)=target+(act(1)-target)*exp(-dt/p.thrusterLag_s);
goals=min(max(command(2:5),-f.angleLimit),f.angleLimit);
for k=1:4
 value=act(k+1);goal=goals(k);error=goal-value;threshold=f.lag*f.rateLimit;
 if abs(error)<=threshold
  next(k+1)=goal+(value-goal)*exp(-dt/f.lag);
 else
  direction=sign(error);saturatedTime=(abs(error)-threshold)/f.rateLimit;
  if dt<=saturatedTime,next(k+1)=value+direction*f.rateLimit*dt;
  else
   boundary=goal-direction*threshold;next(k+1)=goal+(boundary-goal)*exp(-(dt-saturatedTime)/f.lag);
  end
 end
end
end
