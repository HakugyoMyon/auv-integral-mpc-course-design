function m = ablation_diagnostics(run)
steady=run.t>=run.t(end)-50;
m=struct('steadyInnerMeanDeg',rad2deg(mean(run.controlError(steady,1))), ...
 'steadyInnerRmseDeg',rad2deg(sqrt(mean(run.controlError(steady,1).^2))), ...
 'steadyReferenceFilterLagDeg',rad2deg(mean(angle(exp(1i*(run.rawHeading(steady)-run.reference(steady,1)))))), ...
 'etaPeakDegS',0,'etaFreezeFraction',0,'psiPredictionRmseDeg',NaN,'rPredictionRmseDegS',NaN);
if isfield(run.scenario,'headingReference'), m.steadyReferenceFilterLagDeg=NaN; end
if isfield(run,'ablationLog')
 m.etaPeakDegS=rad2deg(max(abs(run.ablationLog(:,1:2)),[],'all'));
 m.etaFreezeFraction=mean(run.ablationLog(:,3));
 e=run.z(2:end,[12 6])-run.nextPrediction(1:end-1,:);
 e(:,1)=angle(exp(1i*e(:,1)));
 m.psiPredictionRmseDeg=rad2deg(sqrt(mean(e(:,1).^2)));
 m.rPredictionRmseDegS=rad2deg(sqrt(mean(e(:,2).^2)));
end
end
