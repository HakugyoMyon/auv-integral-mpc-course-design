function f=auv6_fin_config()
% Preliminary hydrodynamics; geometry from mechanical interface, not CFD fit.
f.rho=1025; f.area=.008925; f.span=.085;
f.AR=f.span^2/f.area; f.efficiency=.8;
% Conservative isolated-fin slope. Very low aspect ratio: sensitivity required.
f.liftSlope=2*pi/(1+2/(f.efficiency*f.AR));
f.alphaLimit=deg2rad(15); f.CD0=.04;
f.angleLimit=deg2rad(25); f.rateLimit=deg2rad(60); f.lag=.15;
f.names={'upper','lower','port','starboard'};
f.root=[-.590,-.690,-.790,-.890;0,0,-.120,.120;-.120,.120,0,0];
f.spanDirection=[0,0,0,0;0,0,-1,1;-1,1,0,0];
f.position=f.root+.03777777778*f.spanDirection;
% Chordwise centre of pressure initially at shaft station; not identified.
f.normal=[0,0,0,0;1,1,0,0;0,0,1,1];
end
