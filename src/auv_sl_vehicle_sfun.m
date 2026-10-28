function auv_sl_vehicle_sfun(block)
block.NumDialogPrms=1;block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=1;block.NumOutputPorts=1;block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=3;block.InputPort(1).DirectFeedthrough=false;
block.OutputPort(1).Dimensions=12;block.NumContStates=12;block.SampleTimes=[0 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('InitializeConditions',@initialize);block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Derivatives',@derivatives);
end
function initialize(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);block.ContStates.Data=c.initial(1:12);
end
function outputs(block)
block.OutputPort(1).Data=block.ContStates.Data;
end
function derivatives(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);a=block.InputPort(1).Data;
dz=remus_reference_rhs(block.CurrentTime,[block.ContStates.Data;a],a,c.P);block.Derivatives.Data=dz(1:12);
end
