function auv_sl_controller_sfun(block)
block.NumDialogPrms=2;block.DialogPrmsTunable={'Nontunable','Nontunable'};
block.NumInputPorts=2;block.NumOutputPorts=2;block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=15;block.InputPort(2).Dimensions=6;
block.InputPort(1).DirectFeedthrough=true;block.InputPort(2).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=3;block.OutputPort(2).Dimensions=18;block.SampleTimes=[.1 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@post);block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Update',@update);
end
function post(block)
block.NumDworks=2;
for k=1:2
 block.Dwork(k).Name=['controllerMemory' num2str(k)];block.Dwork(k).Dimensions=13;
 block.Dwork(k).DatatypeID=0;block.Dwork(k).Complexity='Real';block.Dwork(k).UsedAsDiscState=(k==1);
end
end
function initialize(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);m=control_initial_state(c.initial,c.trim);
block.Dwork(1).Data=auv_sl_memory('pack',m);block.Dwork(2).Data=block.Dwork(1).Data;
end
function outputs(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);mode=block.DialogPrm(2).Data;
m=auv_sl_memory('unpack',block.Dwork(1).Data);z=block.InputPort(1).Data;g=block.InputPort(2).Data;
d=nan(18,1);d(5:7)=0;
if mode==1
 [command,m]=pid_control_step(z,g(1),m,c.config.C,c.P,c.trim,logical(g(6)));
else
 assert(mode==2);
 [rudder,m,info]=active_mpc_heading_step(z,g(1),m,c.config.C,c.P,c.trim,c.config.M,logical(g(6)));
 [command,m]=pid_control_step(z,g(1),m,c.config.C,c.P,c.trim,logical(g(6)),rudder);
 assert(abs(command(1)-rudder)<=1e-8);
 assert(~info.abort,'AUV:SlQP','Three consecutive QP failures in Simulink.');
 d=[info.exitflag;info.solverTime;info.modelAndQpTime;info.violation;double(info.failed); ...
 info.consecutiveFailures;double(info.abort);info.iterations;double(info.boundActive);double(info.rateActive); ...
 info.integralBefore;info.integralAfter;double(info.integralFrozen);info.referenceRate; ...
 info.solverAudit.primaryExitflag;double(info.solverAudit.retryUsed);info.solverAudit.retryExitflag;info.solverAudit.retryStationarity];
end
block.OutputPort(1).Data=command;block.OutputPort(2).Data=d;block.Dwork(2).Data=auv_sl_memory('pack',m);
end
function update(block)
block.Dwork(1).Data=block.Dwork(2).Data;
end
