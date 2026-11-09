function prep=auv12c_prepare(c,seed,surfaceSeed)
a=auv12b_surface_config();stream=RandStream('mt19937ar','Seed',seed);
[cal,bt]=auv_stationary_gyro_calibration(c,stream);state=stream.State;
[m,truth]=auv12b_surface_fixture(c,bt,a,RandStream('mt19937ar','Seed',surfaceSeed));
[noise,estConfig]=auv12b_estimator_parameters(c,a);aligned=auv12b_surface_align(m,cal,noise,estConfig);assert(aligned.valid,'Surface alignment did not pass sensor-quality checks');
prep=struct('calibration',cal,'postCalibrationRandomState',state,'sensors',m,'truth',truth,'alignment',aligned,'configuration',a,'estimatorNoise',noise,'estimatorConfiguration',estConfig);
end
