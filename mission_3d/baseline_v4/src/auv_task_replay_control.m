function report=auv_task_replay_control(file)
% Independent guidance/QP/physical reexecution from archived public inputs.
% Route bookkeeping, estimated states and measured-depth targets are replay
% inputs here; this is NOT an end-to-end fresh stochastic autonomy trial.
z=load(file,'h','m','p','f','trim','plan','prep','routeLog');h=z.h;m=z.m;p=z.p;f=z.f;
memory=d3_initialize_memory(z.trim.control,m);mission=struct('filteredSideslip',0);route=z.plan;
heading=z.prep.alignment.yaw;depth=h.estimate(1,3);x=h.truth(1,:)';refError=0;thrustError=0;finError=0;physicalError=0;minimumRAM=Inf;
for k=1:numel(h.t)
 if mod(k-1,100)==0,[~,sys]=feval('memory');minimumRAM=min(minimumRAM,sys.PhysicalMemory.Available/2^30);assert(minimumRAM>=.75);end
 for j=1:numel(z.routeLog)
  event=z.routeLog{j};if isfinite(event.appliedAt)&&event.appliedAt<=h.t(k)+1e-9
   route=event.route;
   % Operator return starts a new mission object, including its sideslip
   % filter; ordinary remaining-work handoffs retain that filter instead.
   if strcmp(event.kind,'operator_return')&&abs(event.appliedAt-h.t(k))<1e-9,mission.filteredSideslip=0;end
  end
 end
 mission.segment=h.segment(k);mission.progress=h.progress(k);
 status=struct('crossTrack',h.crossTrack(k),'depthTarget',h.altitude(k,3));
 [mission,reference,inputReference,heading,depth]=d16_guidance(mission,route,h.estimate(k,:)',heading,depth,m,z.trim,status);
 refError=max(refError,max(abs(reference(:,1)-h.reference(k,:)')));
 % Terminal zero-duration record contains no consumed QP command.
 if h.appliedCommandDuration_s(k)>0
  [u,memory,info]=d14w_mpc_continuous_step(auv_mpc_state(h.estimate(k,:)'),reference,inputReference,m,memory);assert(info.success);
  thrustError=max(thrustError,abs(u(1)-h.command(k,1)));finError=max(finError,max(abs(u(2:5)-h.command(k,2:5)')));
 end
 physicalError=max(physicalError,max(abs(x-h.truth(k,:)')));
 duration=h.appliedCommandDuration_s(k);steps=round(duration/.01);assert(abs(duration-steps*.01)<1e-8);
 % Physical replay uses the archived applied commands, independently of QP
 % numerical differences, so these two audits do not mask each other.
 command=[auv6_thrust_pwm(h.command(k,1),p);h.command(k,2:5)'];
 for j=1:steps
  a=auv6_actuated_rhs(x,command,p,f);b=auv6_actuated_rhs(x+.005*a,command,p,f);
  d=auv6_actuated_rhs(x+.005*b,command,p,f);e=auv6_actuated_rhs(x+.01*d,command,p,f);
  x=x+.01/6*(a+2*b+2*d+e);x(4:7)=x(4:7)/norm(x(4:7));
 end
end
physicalError=max(physicalError,max(abs(x-h.terminalPhysicalState)));
assert(refError<1e-9&&thrustError<1e-5&&finError<1e-6&&physicalError<1e-10,...
 'Replay mismatch ref=%g thrust=%g fins=%g physics=%g',refError,thrustError,finError,physicalError);
report=struct('passed',true,'control_samples',numel(h.t),'maximum_reference_first_column_error',refError,...
 'maximum_QP_thrust_difference_N',thrustError,'maximum_QP_fin_difference_rad',finError,...
 'maximum_physical_state_difference',physicalError,'minimum_actual_available_RAM_GB',minimumRAM,...
 'scope','Independent guidance/QP and RK4 physical reexecution using recorded route bookkeeping, estimated states and depth targets. Separate raw navigation/payload/map audits required; not a fresh stochastic trial or real-time benchmark.');
fid=fopen(fullfile(fileparts(file),'control_physics_replay.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end
