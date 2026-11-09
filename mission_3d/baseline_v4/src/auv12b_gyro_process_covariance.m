function Q=auv12b_gyro_process_covariance(T,gyroDensity,biasRW)
% Error model: headingErrorDot = -biasError - gyroWhiteNoise;
% biasErrorDot = biasRandomWalk. Integral of exp(F*t)*L*Qc*L'*exp(F*t)'.
q=biasRW^2;Q=[gyroDensity^2*T+q*T^3/3,-q*T^2/2;-q*T^2/2,q*T];
end
