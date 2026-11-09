function st=auv12b_nav_initialize(origin,q,dvl,gyro,bg,poseBiasCov,cal,c,a)
% Independent errors [body position, measured body velocity, Euler angles,bg,ba]
% are mapped into INS-point NED position/velocity and right attitude errors.
R=auv6_rotation(q);angles=auv6_euler_state([zeros(3,1);q;zeros(11,1)]);phi=angles(1);theta=angles(2);
E=[1,0,-sin(theta);0,cos(phi),sin(phi)*cos(theta);0,-sin(phi),cos(phi)*cos(theta)];
r=c.rDVL-c.rINS;vINSbody=dvl-cross(gyro-bg,r);
st=struct('p',origin+R*c.rINS,'v',R*vINSbody,'q',q,'bg',bg,'ba',zeros(3,1));
C=diag([.6^2;.6^2;.06^2;.03^2*ones(3,1);deg2rad(.5)^2;deg2rad(.5)^2;deg2rad(1)^2;...
 (cal.standard_deviation^2+c.gyroBiasRW^2*(a.duration+a.latency))*ones(3,1);.03^2*ones(3,1)]);
idx=[1,2,9,12];C(idx,idx)=poseBiasCov;
T=eye(15);T(1:3,7:9)=-R*auv6_skew(c.rINS)*E;T(4:6,4:6)=R;T(4:6,7:9)=-R*auv6_skew(vINSbody)*E;
T(4:6,10:12)=-R*auv6_skew(r);T(7:9,7:9)=E;
st.P=T*C*T';L=R*auv6_skew(r);st.P(4:6,4:6)=st.P(4:6,4:6)+L*(c.gyroDensity^2/c.dt*eye(3))*L';st.P=(st.P+st.P')/2+1e-14*eye(15);
assert(min(eig(st.P))>0);
end
