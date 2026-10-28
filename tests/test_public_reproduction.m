function test_public_reproduction(includeSimulink)
if nargin<1,includeSimulink=false;end
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);scope=project_environment(); %#ok<NASGU>
verify_source_integrity();cfg=current_control_parameters();trim=compute_trim();scenarios=current_design_scenarios();
for index=[1 2 3 5 11]
 sc=scenarios{index};sc.duration=2;
 for mode=1:2
  M=[];label='PID';if mode==2,M=cfg.M;label='MPC_v3';end
  run=simulate_active_control(cfg.C,trim,sc,cfg.h,M);
  a=load(fullfile(root,'reference_data','v3',[sc.id '_' label '.mat']),'reference');
  assert(max(abs(run.z-a.reference.z(1:numel(run.t),:)),[],'all')<1e-8);
  assert(max(abs(run.command-a.reference.command(1:numel(run.t),:)),[],'all')<1e-8);
 end
end
sc=scenarios{1};sc.duration=2;sensor=sensor_study_configuration('noise_delay',104729);
a=simulate_sensor_control(cfg.C,trim,sc,cfg.h,[],sensor);b=simulate_sensor_control(cfg.C,trim,sc,cfg.h,cfg.M,sensor);
assert(isequal(a.acquisitionNoise,b.acquisitionNoise));
assert(max(abs(a.ey-a.z(:,8)))==0);
if includeSimulink
 for pair=[1 1;1 2;2 2;11 2]'
  result=run_simulink_v3_case(pair(1),pair(2),2);assert(result.summary.passed);
 end
end
fprintf('PUBLIC REPRODUCTION TEST PASSED (Simulink=%d).\n',includeSimulink);
end
