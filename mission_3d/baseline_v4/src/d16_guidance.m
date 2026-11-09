function [mission,ref,ur,nextHeading,nextDepth]=d16_guidance(mission,plan,xhat,previousHeading,previousDepth,m,trim,status)
R=auv6_rotation(xhat(4:7));velocity=R*xhat(8:10);angles=auv6_euler_state(xhat);
beta=atan2(velocity(2),velocity(1))-angles(3);beta=atan2(sin(beta),cos(beta));
mission.filteredSideslip=mission.filteredSideslip+m.Ts/(1+m.Ts)*atan2(sin(beta-mission.filteredSideslip),cos(beta-mission.filteredSideslip));
mission.filteredSideslip=min(max(mission.filteredSideslip,-deg2rad(20)),deg2rad(20));
headings=zeros(1,m.Np);rates=headings;old=previousHeading;
for k=1:m.Np
 [~,gamma]=d16_path_preview(plan,mission.segment,mission.progress,plan.task.speed_m_s*k*m.Ts);
 target=gamma-atan(status.crossTrack/12)-mission.filteredSideslip;
 delta=atan2(sin(target-old),cos(target-old));delta=min(max(delta,-deg2rad(2.5)*m.Ts),deg2rad(2.5)*m.Ts);
 old=old+delta;headings(k)=old;rates(k)=delta/m.Ts;
end
times=(1:m.Np)*m.Ts;depth=previousDepth+min(max(status.depthTarget-previousDepth,-plan.task.depth.maximumRate_m_s*times),plan.task.depth.maximumRate_m_s*times);
ref=repmat(auv_mpc_state(trim.state),1,m.Np);ref(1,:)=depth;ref(4,:)=headings;
% Euler-rate to body-rate reference for phi=0 and fixed trim theta.
ref(8,:)=-rates.*sin(ref(3,:));ref(10,:)=rates.*cos(ref(3,:));
ur=repmat(trim.control,1,m.Np);nextHeading=headings(1);nextDepth=depth(1);
end
