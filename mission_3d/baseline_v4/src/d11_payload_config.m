function c=d11_payload_config(spec)
% Generic geometric image/swath footprint, not a named sonar's performance.
c=spec;c.mount=[.15;0;.14];c.period_s=.2;c.latency_s=.1;
c.rangeStd_m=.025;c.dropoutProbability=.02;c.maximumPlaneDeviation_m=.1;
c.minimumAltitude_m=3;c.maximumTilt_rad=deg2rad(25);c.geometryMargin_m=.35;
a=tan(spec.alongHalfAngle_rad);b=tan(spec.crossHalfAngle_rad);
c.rays=[a,-a,-a,a,0;b,b,-b,-b,0;ones(1,5)];
c.rays=c.rays./vecnorm(c.rays);
end
