function [ sensor_meas,rrw ] = SensorStocModel_Kalibr( sensordata, sig, sig_b, freq, rrw )
%SENSORSTOCMODEL_KALIBR outputs a sensor measurement corrupted by a noise model. 
%This model is a reduced version of the SENSORSTOCMODEL function used in
%the Kalibr toolbox (http://github.com/ethz-asl/kalibr).
%This models only the stochastic component of the sensor. Deterministic
%errors, such as scale and turn-on bias, should be applied to the 
%sensordata input prior to using this function. The following paper is used
%as a reference for this function:
% Paul Furgale, Joern Rehder, Roland Siegwart (2013). Unified Temporal and 
% Spatial Calibration for Multi-Sensor Systems. In Proceedings of the 
% IEEE/RSJ International Conference on Intelligent Robots and Systems 
% (IROS), Tokyo, Japan.
%   Inputs:
%           sensordata = noise free sensor reading. (for example, if a
%                        gyroscope, this would be the true angular velocity
%                        about the sensor). This input also assumes other
%                        deterministic sensor errors such as scale factor 
%                        and turn-on bias are already applied.
%           sig        = angle or velocity random walk (rad/(s*sqrt(Hz)) or
%                        velocity equivalent) [taken from Allan deviation 
%                        curve where tau = 1 
%           sig_b      = rate random walk (rad/(s^2*sqrt(Hz)) or
%                        accelerometer equivalent) [taken from Allan
%                        deviation curve where tau = 3]
%           freq       = Frequency of Sensor Measurements
%           rrw        = Current Value of Rate Random Walk Contribution
%   Outputs:
%           sensor_meas = Output Sensor Measurement
%           rrw = Revised Value of Rate Random Walk Contribution
%   Written by: David Hanley and David Degenhardt (9/29/2017)


%-------------------------------------------------------------------------%
%---------------------------- Rate Random Walk ---------------------------%
%-------------------------------------------------------------------------%
dt = 1/freq;
% Compute next rate random walk state
rrw = rrw + sqrt(dt)*sig_b*randn(1);
%-------------------------------------------------------------------------%
%-------------------------- End Rate Random Walk -------------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%--------------------------- Angle Random Walk ---------------------------%
%-------------------------------------------------------------------------%
% Compute the standard deviation of the angle random walk
sig_arw = sig/sqrt(dt);
% Find angle random walk contribution
arw = sig_arw*randn(1);
%-------------------------------------------------------------------------%
%------------------------- End Angle Random Walk -------------------------%
%-------------------------------------------------------------------------%

%-------------------------------------------------------------------------%
%------------------------- Compute Sensor Output -------------------------%
%-------------------------------------------------------------------------%
% Compute Total Sensor Noise Contribution
sensornoise = arw + rrw;
% Compute Total Sensor Measurement
sensor_meas = sensordata + sensornoise;
%-------------------------------------------------------------------------%
%----------------------- End Compute Sensor Output -----------------------%
%-------------------------------------------------------------------------%
end

