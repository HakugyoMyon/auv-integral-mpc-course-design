function depth=auv6_terrain(north,east)
% Synthetic smooth terrain, depth positive down, global gradient norm <.07.
depth=23+.8*sin(north/35)+.004*north+.4*sin(east/40)...
 -1.5*exp(-((north-170)/45).^2);
end
