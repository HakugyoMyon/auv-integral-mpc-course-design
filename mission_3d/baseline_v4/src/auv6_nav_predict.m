function s=auv6_nav_predict(s,imu,c,propagateCovariance)
% Right-multiplicative 15-error ESKF; nominal point is the INS sensor origin.
if nargin<4,propagateCovariance=true;end
dt=c.dt;w=imu(1:3)-s.bg;f=imu(4:6)-s.ba;
R=auv6_rotation(s.q);
half=[cos(norm(w)*dt/4);.5*dt/2*w];
if norm(w)>1e-10,half=[cos(norm(w)*dt/4);sin(norm(w)*dt/4)*w/norm(w)];end
Rm=auv6_rotation(auv6_qmul(s.q,half));a=Rm*f+c.gravity;
s.p=s.p+s.v*dt+.5*a*dt^2;s.v=s.v+a*dt;
angle=norm(w)*dt;
if angle<1e-10,dq=[1;.5*w*dt];else,dq=[cos(angle/2);sin(angle/2)*w/norm(w)];end
s.q=auv6_qmul(s.q,dq);
if ~propagateCovariance,return;end
F=zeros(15);F(1:3,4:6)=eye(3);F(4:6,7:9)=-R*auv6_skew(f);F(4:6,13:15)=-R;
F(7:9,7:9)=-auv6_skew(w);F(7:9,10:12)=-eye(3);
Phi=eye(15)+F*dt+.5*(F*dt)^2;
Q=zeros(15);Q(4:6,4:6)=c.accelDensity^2*eye(3);Q(7:9,7:9)=c.gyroDensity^2*eye(3);
Q(10:12,10:12)=c.gyroBiasRW^2*eye(3);Q(13:15,13:15)=c.accelBiasRW^2*eye(3);
% Midpoint mapping gives position/velocity noise cross terms.
L=eye(15)+.5*F*dt;s.P=Phi*s.P*Phi'+L*Q*L'*dt;s.P=(s.P+s.P')/2;
end
