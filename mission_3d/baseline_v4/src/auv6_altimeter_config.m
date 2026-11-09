function c=auv6_altimeter_config()
% DVL-mounted generic four-ray bottom ranging model, not vendor firmware.
c.mount=[.15;0;.1511];a=deg2rad(22.5);
c.beams=[sin(a),0,-sin(a),0;0,sin(a),0,-sin(a);cos(a)*ones(1,4)];
c.minRange=.1;c.maxRange=50;c.samplePeriod=.2;c.latency=.1;
c.noiseStd=.025;c.dropoutProbability=.02;c.maxIncidence=deg2rad(60);
c.maxPlaneResidual=.15;c.maxTilt=deg2rad(30);
end
