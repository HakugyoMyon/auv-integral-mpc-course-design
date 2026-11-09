function c=auv6_nav_config()
c.dt=.01;c.lag=.1;c.gravity=[0;0;9.80665];
c.rINS=[-.2;0;.025];c.rDVL=[.15;0;.1511];c.rPressure=[.08;0;-.138];c.rGNSS=[.15;0;-.1485];
c.gyroDensity=deg2rad(.0015);c.accelDensity=.001;
c.gyroBiasRW=deg2rad(.00005);c.accelBiasRW=.0001;
c.gyroBias=deg2rad([.002;-.0015;.003]);c.accelBias=[.015;-.01;.02];
c.dvlStd=.008;c.pressureStd=.03;c.pressureBias=.02;
c.gnssStd=[.6;.6;1.2];c.surfaceThreshold=-.01;
c.initialHeadingError=deg2rad(.5);
end
