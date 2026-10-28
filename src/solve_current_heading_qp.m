function [decision,exitflag,output,audit] = solve_current_heading_qp(qp,M)
% No changes to H,f,A,b or bounds. Retry same convex feasible QP when necessary.
[decision,~,exitflag,output]=quadprog(qp.H,qp.f,qp.A,qp.b,[],[],qp.lb,qp.ub,[],M.options);
audit=struct('primaryExitflag',exitflag,'retryUsed',false,'retryExitflag',NaN, ...
 'retryStationarity',NaN,'zeroSeedViolation',NaN);
if exitflag>0,return;end
zero=zeros(size(qp.f));violation=max([0;qp.A*zero-qp.b;qp.lb-zero;zero-qp.ub]);
[~,notPositiveDefinite]=chol(qp.H);
audit.zeroSeedViolation=violation;
if violation>1e-10 || notPositiveDefinite~=0,return;end
audit.retryUsed=true;
[retryDecision,~,retryFlag,retryOutput,lambda]=quadprog(qp.H,qp.f,qp.A,qp.b,[],[],qp.lb,qp.ub,zero,M.retryOptions);
audit.retryExitflag=retryFlag;
if retryFlag>0 && all(isfinite(retryDecision))
 audit.retryStationarity=norm(qp.H*retryDecision+qp.f+qp.A'*lambda.ineqlin-lambda.lower+lambda.upper,inf);
 decision=retryDecision;exitflag=retryFlag;output=retryOutput;
end
end
