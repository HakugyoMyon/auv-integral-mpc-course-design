function sensor = sensor_study_configuration(condition,seed)
sensor=struct('condition',condition,'seed',seed,'delaySamples',0,'std12',zeros(1,12));
if any(strcmp(condition,{'noise','noise_delay'}))
 sensor.std12=[.01 .01 .01 deg2rad([.02 .02 .02]) .05 .05 .02 deg2rad([.05 .05 .1])];
end
if any(strcmp(condition,{'delay','noise_delay'})),sensor.delaySamples=1;end
assert(any(strcmp(condition,{'ideal','noise','delay','noise_delay'})));
end
