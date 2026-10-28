function auv_sl_actuator_sfun(block)
block.NumDialogPrms=1;block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=1;block.NumOutputPorts=1;block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=3;block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=3;block.NumContStates=3;block.SampleTimes=[0 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('InitializeConditions',@initialize);block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Derivatives',@derivatives);
end
function initialize(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);block.ContStates.Data=c.initial(13:15);
end
function outputs(block)
block.OutputPort(1).Data=block.ContStates.Data;
end
function derivatives(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);block.Derivatives.Data=actuator_rhs(block.ContStates.Data,block.InputPort(1).Data,c.P);
end
