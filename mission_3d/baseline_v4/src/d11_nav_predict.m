function s=d11_nav_predict(s,imu,c)
% 16-error ESKF: original 15 plus persistent magnetic reference offset.
assert(isequal(size(s.P),[16,16])&&isfield(s,'headingBias'));
R=auv6_rotation(s.q);w=imu(1:3)-s.bg;f=imu(4:6)-s.ba;dt=c.dt;
F=zeros(16);F(1:3,4:6)=eye(3);F(4:6,7:9)=-R*auv6_skew(f);
F(4:6,13:15)=-R;F(7:9,7:9)=-auv6_skew(w);F(7:9,10:12)=-eye(3);
Phi=eye(16)+F*dt+.5*(F*dt)^2;
Q=zeros(16);Q(4:6,4:6)=c.accelDensity^2*eye(3);Q(7:9,7:9)=c.gyroDensity^2*eye(3);
Q(10:12,10:12)=c.gyroBiasRW^2*eye(3);Q(13:15,13:15)=c.accelBiasRW^2*eye(3);
Q(16,16)=c.headingAid.biasRW_rad_sqrt_s^2;
L=eye(16)+.5*F*dt;P=Phi*s.P*Phi'+L*Q*L'*dt;
s=auv6_nav_predict(s,imu,c,false);s.P=(P+P')/2;
end
