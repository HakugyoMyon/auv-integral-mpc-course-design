function P = auv_parameters()
% All actuator dynamics below are design assumptions, not OEM measurements.
P.lower = [-deg2rad(20);-deg2rad(20);0];
P.upper = [deg2rad(20);deg2rad(20);1525];
P.rate = [deg2rad(20);deg2rad(20);100];
P.tau = [.2;.2;.5];
P.current = [0;0;0]; % Vc, betaVc, wc; constant within each run
P.kappa = 1;
P.force = @(t) zeros(6,1); % BODY generalized force, N and Nm
end
