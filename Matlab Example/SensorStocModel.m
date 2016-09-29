function [ sensor_meas,bias,rrw ] = SensorStocModel( sensordata,B,K,T,freq,bias,sig_meas,rrw,sig_w )
%SENSORSTOCMODEL outputs a sensor measurement corrupted by a noise model. 
%This models only the stochastic component of the sensor. Deterministic
%errors, such as scale and turn-on bias, should be applied to the 
%sensordata input prior to using this function. The following paper is used
%as a reference for this function:
% P. Petkov and T. Slavov. "Stochastic Modeling of MEMS Inertial Sensors."
% Cybernetics and Information Technologies. Vol. 10. No. 2. pp. 31-40.
% 2010.
%   Inputs:
%           sensordata = noise free sensor reading. (for example, if a
%                        gyroscope, this would be the true angular velocity
%                        about the sensor). This input also assumes other
%                        deterministic sensor errors such as scale factor 
%                        and turn-on bias are already applied.
%           B = Mean Square Value of Bias Instability
%           K = Mean Square Value of Rate Random Walk
%           T = Time Window at Minimum of Allan Variance
%           freq = Frequency of Sensor Measurements
%           bias = Current Value of Bias Instability Contribution
%           sig_meas = Standard Deviation of Total Sensor Measurement
%                      (assuming error of mean zero)
%           rrw = Current Value of Rate Random Walk Contribution
%           sig_w = Standard deviation used in rate random walk
%                   contribution so that low frequency of RRW PSD matches 
%                   low frequency of the PSD from dataset of sensor data
%   Outputs:
%           sensor_meas = Output Sensor Measurement
%           bias = Revised Value of Bias Instability Contribution
%           rrw = Revised Value of Rate Random Walk Contribution
%   Written by: David Hanley (9/28/2016)

%-------------------------------------------------------------------------%
%--------------------------- Bias Instability ----------------------------%
%-------------------------------------------------------------------------%
% Period of sensor measurements
dt = 1/freq;                            
% Set up discrete time bias instability dynamics
a_d = exp(-dt/T);                       
b_d = T - T*exp( - dt / T );
sig_eta = B*sqrt(1-a_d^2)/b_d;
eta = sig_eta*randn(1);
% Compute next bias instability state
bias = a_d*bias + b_d*eta;
%-------------------------------------------------------------------------%
%------------------------- End Bias Instability --------------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%---------------------------- Rate Random Walk ---------------------------%
%-------------------------------------------------------------------------%
% Compute next rate random walk state
rrw = rrw + K*dt*sig_w*randn(1);
%-------------------------------------------------------------------------%
%-------------------------- End Rate Random Walk -------------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%--------------------------- Angle Random Walk ---------------------------%
%-------------------------------------------------------------------------%
% Compute the standard deviation of the angle random walk
sig_arw = sqrt(sig_meas^2 - (K*dt*sig_w)^2 - B^2);
% Find angle random walk contribution
arw = sig_arw*randn(1);
%-------------------------------------------------------------------------%
%------------------------- End Angle Random Walk -------------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%------------------------- Compute Sensor Output -------------------------%
%-------------------------------------------------------------------------%
% Compute Total Sensor Noise Contribution
sensor_noise = bias + arw + rrw;
% Compute Total Sensor Measurement
sensor_meas = sensordata + sensor_noise;
%-------------------------------------------------------------------------%
%----------------------- End Compute Sensor Output -----------------------%
%-------------------------------------------------------------------------%
end

