function report=d16_actuator_audit(file)
z=load(file,'h','p','f','trim');h=z.h;act=z.trim.state(14:18);replayError=0;
for k=1:numel(h.t)
 replayError=max(replayError,max(abs(act-h.estimate(k,14:18)')));
 duration=h.appliedCommandDuration_s(k);steps=round(duration/.01);assert(abs(duration-steps*.01)<1e-8);
 command=[auv6_thrust_pwm(h.command(k,1),z.p);h.command(k,2:5)'];
 for j=1:steps,act=d16_actuator_update(act,command,z.p,z.f,.01);end
end
finError=rad2deg(max(abs(h.estimate(:,15:18)-h.truth(:,15:18)),[],'all'));
assert(replayError<1e-9&&finError<=.05);
report=struct('passed',true,'command_feedback_replay_error',replayError,'sampled_fin_error_deg',finError,...
 'scope','Finite driver replay and offline sampled plant comparison, not hardware feedback or a general uncertainty proof.');
fid=fopen(fullfile(fileparts(file),'actuator_audit.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
end
