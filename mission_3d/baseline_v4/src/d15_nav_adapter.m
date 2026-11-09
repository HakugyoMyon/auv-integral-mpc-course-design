function nav=d15_nav_adapter(state,lever,time,availableTime)
% Marginalize the persistent heading-bias state; do not discard its effect
% on the retained pose covariance or change the original filter itself.
assert(isequal(size(state.P),[16,16]));state.P=state.P(1:15,1:15);
nav=dyn_nav_adapter(state,lever,time,availableTime);
end
