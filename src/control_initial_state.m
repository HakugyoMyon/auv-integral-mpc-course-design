function memory = control_initial_state(z,trim)
memory=struct('integral',zeros(3,1),'rateFiltered',zeros(2,1), ...
 'psiFiltered',z(12),'beta',0,'previousCommand',trim.a);
end
