function pwm=auv6_thrust_pwm(force,p)
% Forward-operation inverse only; no extrapolation outside the static map.
if force<=0,pwm=1500;return;end
idx=p.pwm>1525;forces=p.staticThrust(idx);pwms=p.pwm(idx);
force=min(max(force,forces(1)),forces(end));pwm=interp1(forces,pwms,force,'linear');
end
