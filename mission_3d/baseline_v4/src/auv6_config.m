function p = auv6_config()
% SI units. Body FRD, world NED, origin defined by mechanical interface.
data = fullfile(fileparts(mfilename('fullpath')), 'data');
budget = jsondecode(fileread(fullfile(data,'mass_budget.json')));
s = budget.trimmed_MIL_candidate;
p.mass = s.mass_kg;
p.rG = s.cg_body_FRD_m(:);
p.rB = s.cb_body_FRD_m(:);
p.W = p.mass * budget.g_m_s2;
p.B = budget.fluid_density_kg_m3 * s.displacement_m3 * budget.g_m_s2;
Sg = auv6_skew(p.rG);
J = s.inertia_CG_body_kg_m2 - p.mass * (Sg*Sg);
p.MRB = [p.mass*eye(3), -p.mass*Sg; p.mass*Sg, J];
% Engineering starting values for sensitivity studies. No identification yet.
p.MA = diag([4 26 26 .04 3.5 3.5]);
p.d1 = [2 7 7 .12 .6 .6]';
p.d2 = [8 65 65 .20 3 3]';
p.M = p.MRB + p.MA;
p.thrusterPosition = [-1.205;0;0]; % SW shaft axis Y=1555 mm
p.thrusterLag_s = .25;              % Assumed starting value
tab = readtable(fullfile(data,'T200_static_16V.csv'));
assert(all(ismember({'PWM_us','force_N'},tab.Properties.VariableNames)));
p.pwm = tab.PWM_us(:);
p.staticThrust = tab.force_N(:);
assert(all(diff(p.pwm)>0));
end
