function ledger=auv_task_ledger(ledger,master,xhat,quality,depthSigma,time,observation)
% Persistent original IDs across route changes. No simulator truth input.
if auv_task_is_point_task(master.task.type)
 ledger=auv_task_visit_ledger(ledger,master,xhat,quality,depthSigma,time);
else
 ledger=d11_payload_ledger(ledger,master,observation,quality);
end
end
