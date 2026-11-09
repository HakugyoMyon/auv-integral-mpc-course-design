function p=auv6_candidate_config(budgetFile)
% Explicit opt-in to the latest engineering mass candidate.
p=auv6_config(); b=jsondecode(fileread(budgetFile));s=b.trimmed_MIL_candidate;
p.mass=s.mass_kg;p.rG=s.cg_body_FRD_m(:);p.rB=s.cb_body_FRD_m(:);
p.W=p.mass*b.g_m_s2;p.B=b.fluid_density_kg_m3*s.displacement_m3*b.g_m_s2;
S=auv6_skew(p.rG);J=s.inertia_CG_body_kg_m2-p.mass*S*S;
p.MRB=[p.mass*eye(3),-p.mass*S;p.mass*S,J];p.M=p.MRB+p.MA;
assert(norm(p.M-p.M','fro')<1e-9 && min(eig(p.M))>0);
end
