function s=dyn_contract(kind)
% Interfaces only: task and trajectory generators are not implemented here.
switch kind
 case 'task'
 s=struct('schema','auv.task.v1','frame','NED','id','task01','kind','survey','regionNE',zeros(0,2),'noGoRegionsNE',{cell(0,1)},'targetsNED',zeros(0,3),'priority',zeros(0,1),'depth',14,'altitude',5,'lineSpacing',10,'inspectionOffset',5,'maximumTime',1200,'homeNED',[0;0;12]);
 case 'navigation'
 s=struct('schema','auv.navpose.v1','frame','NED','time',0,'availableTime',0,'positionNED',zeros(3,1),'quaternion',[1;0;0;0],'positionCovNE',eye(2)*.04,'yawVariance',deg2rad(.1)^2);
 case 'trajectory'
 s=struct('schema','auv.trajectory.v1','frame','NED','time',0,'validUntil',0,'sampleTime',zeros(0,1),'positionNED',zeros(0,3),'speed',zeros(0,1),'heading',zeros(0,1),'feasibilityVerified',false,'status','unplanned');
 case 'ledger'
 s=struct('schema','auv.ledger.v1','taskId','task01','goalId','goal01','status','pending','verifiedAt',NaN,'evidence',struct('sampleTime',NaN,'navigationValid',false,'observationValid',false));
 otherwise,error('Unknown contract');
end
end
