function z=auv_mpc_state(x)
% depth,phi,theta,psi,u,v,w,p,q,r,actual_thrust,four_actual_fin_angles
z=[x(3);auv6_euler_state(x)];
end
