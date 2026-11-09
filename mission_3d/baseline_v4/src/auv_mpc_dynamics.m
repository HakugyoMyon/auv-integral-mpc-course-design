function dz=auv_mpc_dynamics(z,u,p,f)
x=auv6_from_euler_state(z(2:15));x(3)=z(1);
command=[auv6_thrust_pwm(u(1),p);u(2:5)];d=auv6_actuated_rhs(x,command,p,f);
ph=z(2);th=z(3);E=[1,sin(ph)*tan(th),cos(ph)*tan(th);0,cos(ph),-sin(ph);0,sin(ph)/cos(th),cos(ph)/cos(th)];
dz=[d(3);E*x(11:13);d(8:18)];
end
