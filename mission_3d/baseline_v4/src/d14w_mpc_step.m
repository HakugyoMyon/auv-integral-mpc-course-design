function [u,memory,info]=d14w_mpc_step(z,reference,inputReference,m,memory,forceFailure)
% All control actions come from this QP. Last feasible command is held once
% on solver failure; no secondary PID/LQR controller is present.
if nargin<6,forceFailure=false;end
x=z-m.z0;
if isempty(memory)
 memory=struct('previousX',x,'previousU',zeros(5,1),'disturbance',zeros(6,1),...
  'warmStart',zeros(5*m.Nc+1,1),'hasPrevious',false,'consecutiveFailures',0);
end
if memory.hasPrevious
 residual=x-(m.Ad*memory.previousX+m.Bd*memory.previousU+m.Ed*memory.disturbance);
 correction=m.Ed(5:10,:)\residual(5:10);
 memory.disturbance=memory.disturbance+m.observerGain*correction;
 memory.disturbance=min(max(memory.disturbance,-m.disturbanceLimit),m.disturbanceLimit);
end
free=m.F*x+m.J*memory.disturbance;
ref=reshape(reference-m.z0,[],1);ur=reshape((inputReference-m.u0)./m.scale,[],1);
previous=[memory.previousU;zeros(5*(m.Nc-1),1)];
gradient=[2*(m.GQ*(free-ref)-m.TR*ur-m.DR*previous);0];
rate=repmat(m.rate*m.Ts./m.scale,m.Nc,1);
upperRate=rate+previous;lowerRate=rate-previous;
attitudeFree=m.C*(free+repmat(m.z0,m.Np,1));
freeRate=m.rateF*x+m.rateJ*memory.disturbance;
b=[upperRate;lowerRate;m.attitudeLimit-attitudeFree;m.attitudeLimit+attitudeFree;...
 m.actualFinRate*ones(4*m.Nc,1)-freeRate;m.actualFinRate*ones(4*m.Nc,1)+freeRate];
alphaFree=m.alphaOrigin+m.alphaSelector*free;
b=[b;m.alphaLinearLimit-alphaFree;m.alphaLinearLimit+alphaFree];
lb=[repmat((m.uMin-m.u0)./m.scale,m.Nc,1);0];ub=[repmat((m.uMax-m.u0)./m.scale,m.Nc,1);Inf];
timer=tic;
if forceFailure,solution=[];flag=-99;output=struct('iterations',0);
else
 if isfield(memory,'solverWarmStart'),ws=memory.solverWarmStart;
 else,ws=optimwarmstart(memory.warmStart,m.warmOptions);end
 [wsOut,~,flag,output]=quadprog(m.H,gradient,m.Aineq,b,[],[],lb,ub,ws);
 solution=wsOut.X;
end
primaryFlag=flag;secondaryUsed=false;violation=Inf;
if ~isempty(solution),violation=max([0; m.Aineq*solution-b;lb-solution;solution-ub]);end
success=flag>0 && ~isempty(solution) && all(isfinite(solution)) && violation<=1e-6;
if ~success && ~forceFailure
 % Retry the identical QP with a different numerical method, not a controller.
 hold=repmat(memory.previousU,m.Nc,1);
 attitudes=m.C*(free+m.G*hold+repmat(m.z0,m.Np,1));
 witness=[hold;max([0;abs(attitudes)-m.attitudeLimit])+1e-8];
 [solution,~,flag,output]=quadprog(m.H,gradient,m.Aineq,b,[],[],lb,ub,[],m.options);
 secondaryUsed=true;violation=Inf;
 if ~isempty(solution),violation=max([0;m.Aineq*solution-b;lb-solution;solution-ub]);end
 success=flag>0 && ~isempty(solution) && all(isfinite(solution)) && violation<=1e-6;
end
elapsed=toc(timer);
if success
 v=solution(1:5);memory.warmStart=solution;memory.consecutiveFailures=0;slack=solution(end);
 if ~secondaryUsed,memory.solverWarmStart=wsOut;elseif isfield(memory,'solverWarmStart'),memory=rmfield(memory,'solverWarmStart');end
else
 v=memory.previousU;memory.consecutiveFailures=memory.consecutiveFailures+1;slack=NaN;
end
u=m.u0+m.scale.*v;
info=struct('success',success,'exitflag',flag,'solve_time_s',elapsed,'iterations',output.iterations,...
 'primary_exitflag',primaryFlag,'secondary_solver_used',secondaryUsed,'constraint_violation',violation,'attitude_slack_rad',slack,'consecutive_failures',memory.consecutiveFailures);
if ~success && ~forceFailure
 hold=repmat(memory.previousU,m.Nc,1);
 attitudes=m.C*(free+m.G*hold+repmat(m.z0,m.Np,1));
 witness=[hold;max([0;abs(attitudes)-m.attitudeLimit])+1e-8];
 info.failure_problem=struct('H',m.H,'gradient',gradient,'A',m.Aineq,'b',b,'lb',lb,'ub',ub,...
  'witness',witness,'witness_violation',max([0;m.Aineq*witness-b;lb-witness;witness-ub]));
end
memory.previousX=x;memory.previousU=v;memory.hasPrevious=true;
end
