function clearance=auv6_clearance_bound(x,terrain)
% Offline conservative vertical clearance for a box enclosing the entire AUV.
% x in [-1.30,.65], |y|,|z|<=.205 m; terrain Lipschitz bound .07.
[a,b,c]=ndgrid([-1.30,.65],[-.205,.205],[-.205,.205]);
offset=auv6_rotation(x(4:7))*[a(:)';b(:)';c(:)'];
clearance=terrain(x(1),x(2))-x(3)-max(offset(3,:))...
 -.07*max(vecnorm(offset(1:2,:),2,1));
end
