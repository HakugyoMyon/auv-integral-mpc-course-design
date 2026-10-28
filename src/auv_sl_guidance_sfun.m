function auv_sl_guidance_sfun(block)
block.NumDialogPrms=1;block.DialogPrmsTunable={'Nontunable'};
block.NumInputPorts=1;block.NumOutputPorts=1;block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=15;block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=6;block.SampleTimes=[.1 0];
block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('PostPropagationSetup',@post);block.RegBlockMethod('InitializeConditions',@initialize);
block.RegBlockMethod('Outputs',@outputs);block.RegBlockMethod('Update',@update);
end
function post(block)
block.NumDworks=2;
for k=1:2
 block.Dwork(k).Name=['guidanceMemory' num2str(k)];block.Dwork(k).Dimensions=2;
 block.Dwork(k).DatatypeID=0;block.Dwork(k).Complexity='Real';block.Dwork(k).UsedAsDiscState=(k==1);
end
end
function initialize(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);block.Dwork(1).Data=[c.initial(12);0];block.Dwork(2).Data=block.Dwork(1).Data;
end
function outputs(block)
c=auv_sl_configuration(block.DialogPrm(1).Data);a=block.Dwork(1).Data;
m=struct('psiFiltered',a(1),'beta',a(2));z=block.InputPort(1).Data;
[reference,m,g]=ablation_guidance(z(1:12),c.scenario,m,c.config.C);
block.OutputPort(1).Data=[reference;g.ey;g.beta;g.rawHeading;g.groundSpeed;double(g.lowSpeed)];
block.Dwork(2).Data=[m.psiFiltered;m.beta];
end
function update(block)
block.Dwork(1).Data=block.Dwork(2).Data;
end
