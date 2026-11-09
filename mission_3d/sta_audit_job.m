function sta_audit_job(file)
folder=fileparts(file);
try
 a=sta_static_audit(file,struct('physicalReplay',true,'controlReplay',true));assert(a.passed,'Static archive audit returned false.');
 a=sta_nav_envelope_audit(file);assert(a.passed,'Navigation envelope violation.');
a=sta_body_prediction_audit(file);assert(a.passed,'Full body prediction tube violation.');
a=sta_prediction_tracking_audit(file);assert(a.passed,'Prediction tracking budget exceeded.');
d16_actuator_audit(file);d11_audit_heading_run(file);d11_audit_heading_innovations(file);
 q=load(file,'plan');if auv_task_is_point_task(q.plan.task.type),auv_task_audit_visit(file);else,auv_task_audit_payload(file);end
 fid=fopen(fullfile(folder,'AUDITS_DONE.txt'),'w');fprintf(fid,'Static map, control/QP, independent physics, actuator, raw navigation and original ledger audits executed. Read each JSON passed flag.');fclose(fid);
catch err
 fid=fopen(fullfile(folder,'AUDITS_ERROR.txt'),'w');fprintf(fid,'%s',getReport(err,'extended'));fclose(fid);
end
end



