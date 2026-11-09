function clear=d11_chart_clear(segments,chart)
% Nominal geometric candidates only; no unobserved dynamic environment input.
clear=true;spacing=.5;inflation=chart.clearance_m+spacing/2;
for k=1:numel(segments)
 s=segments(k);samples=unique([0:spacing:s.length,s.length]);[p,~]=d11_path_eval(s,samples);
 b=chart.operatingBounds;
 if any(p(:,1)<b(1)+inflation|p(:,1)>b(2)-inflation|p(:,2)<b(3)+inflation|p(:,2)>b(4)-inflation)
  clear=false;return;
 end
 for j=1:size(chart.knownCircles,1)
  circle=chart.knownCircles(j,:);
  if any(vecnorm(p-circle(1:2),2,2)<=circle(3)+inflation),clear=false;return;end
 end
end
end
