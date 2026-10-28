function ctx = auv_sl_configuration(caseIndex)
% Immutable cache rebuilt solely from explicit case index and frozen functions.
persistent contexts
assert(isscalar(caseIndex) && caseIndex==fix(caseIndex) && caseIndex>=1 && caseIndex<=12);
if isempty(contexts),contexts=cell(12,1);end
if isempty(contexts{caseIndex})
 config=current_control_parameters();assert(strcmp(config.version,'MPC_v3_integral_reference_hold'));
 scenarios=current_design_scenarios();scenario=scenarios{caseIndex};trim=compute_trim();P=auv_parameters();
 P.current=scenario.current;P.kappa=scenario.kappa;P.force=scenario.force;
 P.upper(1)=scenario.rudderMax;P.lower(1)=-scenario.rudderMax;P.rate(1)=scenario.rudderRate;
 initial=[trim.x;trim.a];initial(7)=0;initial(8)=scenario.initialError;initial(12)=scenario.headingOffset;
 contexts{caseIndex}=struct('config',config,'scenario',scenario,'trim',trim,'P',P,'initial',initial);
end
ctx=contexts{caseIndex};
end
