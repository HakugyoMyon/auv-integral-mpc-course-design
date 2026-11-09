function result=d14w_fresh_gate(context,reference,inputReference,plannedSpeed,m,p,f,step)
% One current-state QP plus nonlinear rollout of its blocked open-loop plan.
% Conditional model preview, not future closed-loop/sensor truth.
if nargin<8,step=.05;end
clock=tic;result=struct('accepted',false,'reason','','firstCommand',[],'peak_raw_alpha_deg',NaN,'peak_actual_fin_deg',NaN,'peak_actual_fin_rate_deg_s',NaN,...
 'peak_roll_deg',NaN,'peak_pitch_deg',NaN,'fin_work_margin_deg',NaN,'fin_mechanical_margin_deg',NaN,...
 'command_rate_margin_deg_s',NaN,'thrust_margin_N',NaN,'QP_success',false,'preview_time_s',NaN,'contextRevision',context.revision,...
 'state_time',context.time,'actuatorSource',context.actuatorSource,'transitionClass','','predicted_terminal_state',nan(18,1),'nextMemory',[],...
 'scope','Explicit 3D-reference still-water open-loop MPC/actuator model preview. Not a qualified library transition, execution tube or obstacle safety certificate. First command and updated MPC memory must be consumed together once.');
assert(strcmp(context.schema,'auv.execution_state.v1'));
if ~context.commandHistoryValid || context.lastCommandTime>context.time || context.time-context.lastCommandTime>.2,result.reason='missing_or_stale_command_history';finish();return;end
if context.actuatorTime>context.time || context.time-context.actuatorTime>.2,result.reason='stale_or_future_actuator_state';finish();return;end
if ~any(strcmp(context.actuatorSource,{'command_model','measured'})),result.reason='unknown_actuator_source';finish();return;end
if ~strcmp(context.flowAssumption,'still_water'),result.reason='unqualified_flow_assumption';finish();return;end
if any(abs(context.finHealth-1)>1e-12)||abs(context.thrustHealth-1)>1e-12,result.reason='unqualified_actuator_health';finish();return;end
x=context.state;assert(numel(x)==18 && all(isfinite(x)));
R=auv6_rotation(x(4:7));speed=norm(R(1:2,:)*x(8:10));
if ~isequal(size(reference),[15,m.Np])||~isequal(size(inputReference),[5,m.Np])||~all(isfinite(reference),'all')||~all(isfinite(inputReference),'all'),result.reason='invalid_reference';finish();return;end
if plannedSpeed~=.8||abs(speed-plannedSpeed)>.05,result.reason='speed_outside_candidate_domain';finish();return;end
if max(abs(x(15:18)))>f.angleLimit,result.reason='mechanical_fin_limit';finish();return;end
if max(abs(x(15:18)))>deg2rad(8.05),result.reason='outside_working_fin_domain';finish();return;end
if x(14)<0 || x(14)>max(p.staticThrust),result.reason='thrust_state_outside_domain';finish();return;end
euler=auv6_euler_state(x);beta=atan2(x(9),x(8));
if abs(beta)>deg2rad(5)||abs(euler(1))>deg2rad(15)||abs(euler(2))>deg2rad(20),result.reason='attitude_or_sideslip_outside_domain';finish();return;end
result.transitionClass='explicit_3d_reference_model_preview_only';
result.libraryTransitionQualified=false;
result.fin_work_margin_deg=8-rad2deg(max(abs(x(15:18))));
result.fin_mechanical_margin_deg=rad2deg(f.angleLimit-max(abs(x(15:18))));
[u,memory,info]=d14w_mpc_continuous_step(auv_mpc_state(x),reference,inputReference,m,context.memory);
result.QP_success=info.success;result.currentInfo=info;
if ~info.success,result.reason='current_state_QP_infeasible';finish();return;end
solution=memory.warmStart(1:5*m.Nc);normalized=reshape(m.T*solution,5,[]);
sequence=m.u0+m.scale.*normalized;
previous=m.u0+m.scale.*context.memory.previousU;
result.firstCommand=u;result.command_rate_margin_deg_s=30-rad2deg(max(abs(u(2:5)-previous(2:5)))/m.Ts);
result.thrust_margin_N=max(p.staticThrust)-max(sequence(1,:));
cache=d8_preview_cache(p,f);prediction=zeros(18,m.Np+1);prediction(:,1)=x;
alphaMax=0;rateMax=0;rollMax=0;pitchMax=0;finMax=max(abs(x(15:18)));
for k=1:m.Np
 command=[auv6_thrust_pwm(sequence(1,k),p);sequence(2:5,k)];
 targetThrust=interp1(p.pwm,p.staticThrust,command(1),'linear');if abs(command(1)-1500)<=25,targetThrust=0;end
 n=max(1,ceil(m.Ts/step));dt=m.Ts/n;
 for j=1:n
  [a,alphaA,rateA]=d8_preview_rhs(x,targetThrust,command(2:5),p,f,cache);
  [b,alphaB]=d8_preview_rhs(x+dt*a/2,targetThrust,command(2:5),p,f,cache);[d,alphaD]=d8_preview_rhs(x+dt*b/2,targetThrust,command(2:5),p,f,cache);[e,alphaE]=d8_preview_rhs(x+dt*d,targetThrust,command(2:5),p,f,cache);
  x=x+dt/6*(a+2*b+2*d+e);x(4:7)=x(4:7)/norm(x(4:7));
  [~,alphaAfter,rateAfter]=d8_preview_rhs(x,targetThrust,command(2:5),p,f,cache);
  alphaMax=max([alphaMax;abs(alphaA);abs(alphaB);abs(alphaD);abs(alphaE);abs(alphaAfter)]);
  finMax=max(finMax,max(abs(x(15:18))));
  rateMax=max([rateMax;abs(rateA);abs(rateAfter)]);
  angles=auv6_euler_state(x);rollMax=max(rollMax,abs(angles(1)));pitchMax=max(pitchMax,abs(angles(2)));
 end
 prediction(:,k+1)=x;
end
result.predictedTrajectory=prediction;result.predictionTimes=(0:m.Np)*m.Ts;
result.peak_raw_alpha_deg=rad2deg(alphaMax);result.peak_actual_fin_rate_deg_s=rad2deg(rateMax);
result.peak_actual_fin_deg=rad2deg(finMax);
result.peak_roll_deg=rad2deg(rollMax);result.peak_pitch_deg=rad2deg(pitchMax);
result.accepted=alphaMax+deg2rad(context.actuatorUncertainty_deg)<=deg2rad(14.75)&&rollMax<=deg2rad(15)&&pitchMax<=deg2rad(20)&&rateMax<=f.rateLimit+1e-6&&finMax<=deg2rad(8.05);
if result.accepted,result.reason='conditional_actuator_preview_passed';else,result.reason='predicted_physical_envelope_rejected';end
result.predicted_terminal_state=x;result.nextMemory=memory;
finish();
 function finish()
 result.preview_time_s=toc(clock);
 end
end

