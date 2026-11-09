function x=auv6_nav_output(s,gyro,actuator,c)
% Convert INS-point navigation to the declared body reference origin.
R=auv6_rotation(s.q);omega=gyro-s.bg;
x=zeros(18,1);x(1:3)=s.p-R*c.rINS;x(4:7)=s.q;
x(8:10)=R'*s.v-cross(omega,c.rINS);x(11:13)=omega;x(14:18)=actuator;
end
