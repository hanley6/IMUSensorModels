function [ sensor_meas ] = SensorDetModel( sensordata,S,Bf )
%SENSORDETMODEL Outputs corrupted sensor measurement due to deterministic 
%errors. This approach ignore cross coupling effects, g-sensitive bias, and
%vibro-pendulous error since the input is only the error free sensor 
%output. This approach is based on:
%D. Titterton and J. Weston. "Strapdown Inertial Navigation Technology." 
%The Institution of Engineering and Technology. 2nd Edition. 2004.
%	Inputs:
%           sensordata = noise free sensor reading. (for example, if a
%                        gyroscope, this would be the true angular velocity
%                        about the sensor).
%           S = Current Value of Bias Instability Contribution
%           Bf = Current Value of Rate Random Walk Contribution
%   Outputs:
%           sensor_meas = Output Sensor Measurement with Deterministic
%                         Errors
%   Written by: David Hanley (9/29/2016)

% Compute corrupted sensor measurement due to deterministic errors
sensor_meas = (1.0 + S)*sensordata + Bf;
end

