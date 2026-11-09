function yes=auv_task_is_point_task(type)
yes=any(strcmp(type,{'visit','transit','return'}));
end
